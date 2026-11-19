#!/usr/bin/env python3
"""task.py 의 Task 포함 관계를 docs/task_graph.md (Mermaid) 로 생성한다.

task.py 를 import 하지 않고 AST 만 읽으므로 ROS 환경 없이 실행된다.

    python3 rby1_planner/scripts/gen_task_graph.py          # 문서 갱신
    python3 rby1_planner/scripts/gen_task_graph.py --check  # 문서가 최신인지 확인 (아니면 exit 1)

규칙
- Task 함수: 반환 타입에 Task / RunnableTaskDefinition 이 들어간 최상위 함수.
- 포함 관계: Task 함수 안에서 다른 Task 함수를 호출하면 (task.extend(f(...)) 또는 return f(...)) 간선.
- 레벨: 하위 Task 가 없으면 L0, 아니면 1 + 하위 Task 레벨의 최댓값.
- 등록 Task: build_tasks() 딕셔너리 값으로 호출되는 함수 (Scenario UI 에 보인다).
"""
from __future__ import annotations

import argparse
import ast
import sys
from dataclasses import dataclass, field
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parents[1]
TASK_PY = PKG_ROOT / 'rby1_planner' / 'task.py'
OUT_MD = PKG_ROOT / 'docs' / 'task_graph.md'

TASK_RETURN_TYPES = ('Task', 'RunnableTaskDefinition')
TEST_PREFIX = 'test'
ARG_MAX_LEN = 48


@dataclass
class Step:
    """Task 함수 본문의 한 단계 (소스 순서)."""
    kind: str                 # 'sub' (하위 Task 호출) | 'prim' (task.<method>)
    name: str
    args: str
    note: str = ''            # '조건부' / '반복' 등


@dataclass
class TaskFn:
    name: str
    lineno: int
    doc: str
    steps: list[Step] = field(default_factory=list)

    @property
    def callees(self) -> list[str]:
        seen: list[str] = []
        for s in self.steps:
            if s.kind == 'sub' and s.name not in seen:
                seen.append(s.name)
        return seen


def _short(text: str, limit: int = ARG_MAX_LEN) -> str:
    text = ' '.join(text.split())
    return text if len(text) <= limit else text[: limit - 1] + '…'


def _call_args(call: ast.Call) -> str:
    parts = [ast.unparse(a) for a in call.args]
    parts += [f'{k.arg}={ast.unparse(k.value)}' for k in call.keywords if k.arg]
    return _short(', '.join(parts))


def _is_task_fn(fn: ast.FunctionDef) -> bool:
    if fn.returns is None or fn.name == 'build_tasks':
        return False
    ret = ast.unparse(fn.returns)
    return any(t in ret for t in TASK_RETURN_TYPES)


def _task_vars(fn: ast.FunctionDef) -> set[str]:
    """`x = Task(...)` 로 만든 변수 이름."""
    names: set[str] = set()
    for node in ast.walk(fn):
        if (isinstance(node, ast.Assign) and isinstance(node.value, ast.Call)
                and isinstance(node.value.func, ast.Name) and node.value.func.id == 'Task'):
            names |= {t.id for t in node.targets if isinstance(t, ast.Name)}
    return names


def _collect_steps(fn: ast.FunctionDef, task_names: set[str]) -> list[Step]:
    tvars = _task_vars(fn)
    steps: list[Step] = []

    def calls_in_order(node: ast.AST) -> list[ast.Call]:
        calls = [n for n in ast.walk(node) if isinstance(n, ast.Call)]
        return sorted(calls, key=lambda c: (c.lineno, c.col_offset))

    def visit(stmts: list[ast.stmt], note: str) -> None:
        for st in stmts:
            if isinstance(st, ast.If):
                visit(st.body, '조건부')
                visit(st.orelse, '조건부')
            elif isinstance(st, (ast.For, ast.While)):
                visit(st.body, '반복')
            elif isinstance(st, ast.Try):
                visit(st.body, note)
                for h in st.handlers:
                    visit(h.body, '예외 시')
                visit(st.finalbody, note)
            elif isinstance(st, (ast.With,)):
                visit(st.body, note)
            elif isinstance(st, (ast.FunctionDef, ast.ClassDef)):
                continue
            else:
                for c in calls_in_order(st):
                    f = c.func
                    if isinstance(f, ast.Name) and f.id in task_names and f.id != fn.name:
                        steps.append(Step('sub', f.id, _call_args(c), note))
                    elif (isinstance(f, ast.Attribute) and isinstance(f.value, ast.Name)
                          and f.value.id in tvars and f.attr not in ('extend', 'build')):
                        steps.append(Step('prim', f.attr, _call_args(c), note))

    visit(fn.body, '')
    return steps


def _registered(tree: ast.Module) -> list[tuple[str, str, str]]:
    """build_tasks() 의 (UI 이름, 함수 이름, 인자) 목록."""
    out: list[tuple[str, str, str]] = []
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == 'build_tasks':
            for d in ast.walk(node):
                if isinstance(d, ast.Dict):
                    for k, v in zip(d.keys, d.values):
                        if (isinstance(k, ast.Constant) and isinstance(v, ast.Call)
                                and isinstance(v.func, ast.Name)):
                            out.append((k.value, v.func.id, _call_args(v)))
    return out


def analyze(source: str) -> tuple[dict[str, TaskFn], list[tuple[str, str, str]]]:
    tree = ast.parse(source)
    fns = [n for n in tree.body if isinstance(n, ast.FunctionDef) and _is_task_fn(n)]
    names = {f.name for f in fns}
    tasks: dict[str, TaskFn] = {}
    for f in fns:
        doc = (ast.get_docstring(f) or '').strip().splitlines()
        tasks[f.name] = TaskFn(f.name, f.lineno, doc[0] if doc else '', _collect_steps(f, names))
    return tasks, _registered(tree)


def levels(tasks: dict[str, TaskFn]) -> dict[str, int]:
    memo: dict[str, int] = {}

    def lv(name: str, stack: tuple[str, ...] = ()) -> int:
        if name in memo:
            return memo[name]
        if name in stack:  # 순환 방지
            return 0
        subs = tasks[name].callees
        memo[name] = 0 if not subs else 1 + max(lv(s, stack + (name,)) for s in subs)
        return memo[name]

    for n in tasks:
        lv(n)
    return memo


# ---------------------------------------------------------------------------
# Markdown 출력
# ---------------------------------------------------------------------------
LEVEL_TITLE = {0: 'L0 · 기본 Task (하위 Task 없음)'}


# 화면 폭에 맞춰 축소하지 않고 원래 크기로 그린다 (넘치면 가로 스크롤).
MERMAID_INIT = '%%{init: {"flowchart": {"useMaxWidth": false, "nodeSpacing": 30, "rankSpacing": 50}}}%%'
SCENARIO_MIN_NODES = 4


def _mermaid_label(text: str) -> str:
    return text.replace('"', "'").replace('|', '/')


def _class_lines(nodes: list[str], reg_fns: set[str], used: set[str]) -> list[str]:
    lines = [
        '  classDef registered fill:#dbeafe,stroke:#2563eb,color:#111;',
        '  classDef internal fill:#f3f4f6,stroke:#6b7280,stroke-dasharray:4 3,color:#111;',
        '  classDef unused fill:#fee2e2,stroke:#dc2626,stroke-dasharray:4 3,color:#111;',
    ]
    for cls, members in (
        ('registered', [n for n in nodes if n in reg_fns]),
        ('internal', [n for n in nodes if n not in reg_fns and n in used]),
        ('unused', [n for n in nodes if n not in reg_fns and n not in used]),
    ):
        if members:
            lines.append(f'  class {",".join(members)} {cls};')
    return lines


def connected(tasks: dict[str, TaskFn], used: set[str]) -> list[str]:
    """간선이 하나라도 있는 Task."""
    return [n for n, t in tasks.items() if t.callees or n in used]


def render_graph(tasks: dict[str, TaskFn], lv: dict[str, int], reg_fns: set[str],
                 used: set[str]) -> list[str]:
    """전체 개요: 간선 있는 Task 만, 라벨 없이, 레벨별 열."""
    nodes = connected(tasks, used)
    lines = ['```mermaid', MERMAID_INIT, 'flowchart LR']
    tests = [n for n in nodes if n.startswith(TEST_PREFIX)]
    max_lv = max(lv.values(), default=0)
    # 상위 레벨을 왼쪽에 둔다 (LR: 먼저 선언한 subgraph 가 왼쪽).
    groups: list[tuple[str, str, list[str]]] = [('TESTS', '테스트', tests)]
    for level in range(max_lv, -1, -1):
        members = [n for n in nodes if lv[n] == level and n not in tests]
        groups.append((f'L{level}', LEVEL_TITLE.get(level, f'L{level}'), members))
    for gid, title, members in groups:
        if not members:
            continue
        lines.append(f'  subgraph {gid}["{title}"]')
        lines += [f'    {n}["{n}"]' for n in members]
        lines.append('  end')
    for n in nodes:
        for sub in tasks[n].callees:
            lines.append(f'  {n} --> {sub}')
    lines += _class_lines(nodes, reg_fns, used)
    lines.append('```')
    return lines


def descendants(tasks: dict[str, TaskFn], root: str) -> list[str]:
    out: list[str] = []
    todo = [root]
    while todo:
        n = todo.pop(0)
        if n in out:
            continue
        out.append(n)
        todo += tasks[n].callees
    return out


def scenario_roots(tasks: dict[str, TaskFn], lv: dict[str, int], used: set[str]) -> list[str]:
    """다른 Task 에 포함되지 않는 조합 Task (테스트 제외). 하위 구조가 같은 것은 하나만."""
    roots: list[str] = []
    seen_sig: set[tuple] = set()
    for n in sorted(tasks, key=lambda n: (-lv[n], tasks[n].lineno)):
        if n in used or n.startswith(TEST_PREFIX):
            continue
        if len(descendants(tasks, n)) < SCENARIO_MIN_NODES:
            continue
        sig = tuple((s.kind, s.name, s.args) for s in tasks[n].steps)
        if sig in seen_sig:
            continue
        seen_sig.add(sig)
        roots.append(n)
    return roots


def render_scenario(tasks: dict[str, TaskFn], root: str, reg_fns: set[str],
                    used: set[str]) -> list[str]:
    """root 아래만 위->아래로. 간선 라벨 = 부모 안에서의 호출 순서 (+ 짧은 인자)."""
    nodes = descendants(tasks, root)
    lines = ['```mermaid', MERMAID_INIT, 'flowchart TD']
    lines += [f'  {n}["{n}"]' for n in nodes]
    for n in nodes:
        order: dict[str, list[str]] = {}
        i = 0
        for s in tasks[n].steps:
            if s.kind != 'sub':
                continue
            i += 1
            arg = f': {s.args}' if s.args and len(s.args) <= 18 else ''
            order.setdefault(s.name, []).append(f'{i}{arg}')
        for sub, labels in order.items():
            lines.append(f'  {n} -->|"{_mermaid_label(" / ".join(labels))}"| {sub}')
    lines += _class_lines(nodes, reg_fns, used)
    lines.append('```')
    return lines


def render_tree(tasks: dict[str, TaskFn], lv: dict[str, int], name: str, depth: int,
                out: list[str], expanded: set[str]) -> None:
    """name 의 단계를 펼친다. L0 Task 는 한 줄, 같은 트리에서 이미 펼친 Task 는 다시 펼치지 않는다."""
    indent = '  ' * depth
    for s in tasks[name].steps:
        note = f' _({s.note})_' if s.note else ''
        if s.kind == 'sub':
            again = s.name in expanded and lv[s.name] > 0
            mark = ' ↑ 위에서 펼침' if again else ''
            out.append(f'{indent}- **{s.name}**`({s.args})`{note}{mark}')
            if lv[s.name] > 0 and not again:
                expanded.add(s.name)
                render_tree(tasks, lv, s.name, depth + 1, out, expanded)
        else:
            out.append(f'{indent}- `{s.name}({s.args})`{note}')


def render(source: str) -> str:
    tasks, reg = analyze(source)
    lv = levels(tasks)
    reg_fns = {fn for _, fn, _ in reg}
    used = {c for t in tasks.values() for c in t.callees}
    unused = [n for n in tasks if n not in reg_fns and n not in used]

    md: list[str] = [
        '# task.py Task 관계도',
        '',
        '> 자동 생성 문서 — 직접 고치지 말 것.',
        '> 원본: `rby1_planner/task.py`. 갱신: `python3 rby1_planner/scripts/gen_task_graph.py`',
        '> (최신 여부 확인: `--check`). Task 별 설명은 [TASKS.md](../TASKS.md).',
        '',
        '## 읽는 법',
        '',
        '- 화살표 `A --> B` : A 가 B 를 **포함**한다 (`task.extend(B(...))` 또는 `return B(...)`).',
        '- 레벨: 하위 Task 가 없으면 L0, 아니면 1 + 가장 깊은 하위 Task 의 레벨. 왼쪽일수록 상위.',
        '- 시나리오별 그림의 화살표 라벨 = 부모 Task 안에서 몇 번째로 실행되는 하위 Task 인지 (+ 짧은 인자).',
        '- 그림은 원래 크기로 그린다. 화면보다 크면 가로 스크롤.',
        '- 색: 파랑 = `build_tasks()` 에 등록 (Scenario UI 에 보임),'
        ' 회색 점선 = 다른 Task 안에서만 사용, 빨강 점선 = 어디서도 안 씀 (정리 후보).',
        '',
        '## 전체 포함 관계 (개요)',
        '',
        *render_graph(tasks, lv, reg_fns, used),
        '',
        '포함 관계가 없는 단독 Task: '
        + ', '.join(f'`{n}`' for n in tasks if n not in connected(tasks, used)),
        '',
        '## 시나리오별 포함 관계',
        '',
    ]
    for root in scenario_roots(tasks, lv, used):
        same = [n for n in tasks if n != root and n not in used and not n.startswith(TEST_PREFIX)
                and tasks[n].steps and [(s.kind, s.name, s.args) for s in tasks[n].steps]
                == [(s.kind, s.name, s.args) for s in tasks[root].steps]]
        extra = f' (= {", ".join(f"`{n}`" for n in same)})' if same else ''
        md += [f'### {root}{extra}', '', *render_scenario(tasks, root, reg_fns, used), '']
    md += [
        '## 등록 Task 와 함수',
        '',
        '| UI 이름 | 함수 | 레벨 | 직접 포함하는 Task |',
        '|---|---|---|---|',
    ]
    for ui, fn, args in reg:
        if fn not in tasks:
            md.append(f'| `{ui}` | `{fn}({args})` | ? | (Task 함수 아님) |')
            continue
        subs = ', '.join(f'`{c}`' for c in tasks[fn].callees) or '-'
        md.append(f'| `{ui}` | [`{fn}`](../rby1_planner/task.py#L{tasks[fn].lineno}) | L{lv[fn]} | {subs} |')

    internal = [n for n in tasks if n not in reg_fns and n in used]
    md += ['', '## 등록 안 된 Task 함수', '', '| 함수 | 레벨 | 사용처 |', '|---|---|---|']
    for n in internal + unused:
        callers = ', '.join(f'`{c}`' for c, t in tasks.items() if n in t.callees) or '**없음**'
        md.append(f'| [`{n}`](../rby1_planner/task.py#L{tasks[n].lineno}) | L{lv[n]} | {callers} |')

    md += [
        '',
        '## 실행 순서 (조합 Task 펼침)',
        '',
        '등록 Task 중 하위 Task 를 포함하는 것만. **굵게** = 하위 Task, `코드` = 기본 명령'
        ' (`task.<method>`). L0 Task 는 펼치지 않는다 (내용은 task.py / TASKS.md).',
        '_(조건부)_ / _(반복)_ 은 if / for 안의 단계, ↑ 는 같은 트리에서 이미 펼친 Task.',
        '',
    ]
    seen: set[str] = set()
    composites = sorted((fn for _, fn, _ in reg if fn in tasks and lv[fn] > 0),
                        key=lambda n: (-lv[n], tasks[n].lineno))
    for fn in composites:
        if fn in seen:
            continue
        seen.add(fn)
        doc = f' — {tasks[fn].doc}' if tasks[fn].doc else ''
        md += ['<details>', f'<summary><b>{fn}</b> (L{lv[fn]}){doc}</summary>', '']
        render_tree(tasks, lv, fn, 0, md, set())
        md += ['', '</details>', '']

    return '\n'.join(md).rstrip() + '\n'


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--check', action='store_true', help='문서가 최신이 아니면 exit 1')
    ap.add_argument('--stdout', action='store_true', help='파일 대신 표준출력')
    a = ap.parse_args()

    text = render(TASK_PY.read_text(encoding='utf-8'))
    if a.stdout:
        sys.stdout.write(text)
        return 0
    if a.check:
        current = OUT_MD.read_text(encoding='utf-8') if OUT_MD.exists() else ''
        if current != text:
            print(f'{OUT_MD} 가 task.py 와 다르다. gen_task_graph.py 를 다시 실행할 것.', file=sys.stderr)
            return 1
        print('up to date')
        return 0
    OUT_MD.parent.mkdir(parents=True, exist_ok=True)
    OUT_MD.write_text(text, encoding='utf-8')
    print(f'wrote {OUT_MD}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
