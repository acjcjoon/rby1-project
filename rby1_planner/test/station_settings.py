"""테스트용 settings.yaml 생성 도우미 (stations 구조)."""

# 머리 pitch 절대각 기본값: 이동 중(head_pitch_deg.move)과 지점별(points.<지점>.head_pitch_deg)
HEAD_PITCH_DEG = {'move': 2.0}
POINT_HEAD_PITCH_DEG = 32.0


def stations_settings(locations: dict, current_location: object = None) -> dict:
    """단순 지점 목록을 plain 장비 설정으로 바꾼다.

    locations 예: {'A': {'position': [0, 0], 'side': 'left', 'tag_2': {'offset': [0, 0, 0]}}}
    태그 키가 없으면 place.tag 는 null 이다. head_pitch_deg 를 생략하면 POINT_HEAD_PITCH_DEG.
    """
    stations = {}
    for name, location in locations.items():
        tags = [key for key in location if str(key).startswith('tag_')]
        tag = tags[0] if tags else None
        place = {'point': 'place', 'tag': tag}
        if tag is not None:
            place['offset'] = location[tag]['offset']
        stations[name] = {
            'kind': 'plain',
            'points': {'place': {
                'position': location['position'],
                'side': location['side'],
                'head_pitch_deg': location.get('head_pitch_deg', POINT_HEAD_PITCH_DEG),
            }},
            'place': place,
        }
    settings = {'stations': stations, 'head_pitch_deg': dict(HEAD_PITCH_DEG)}
    if current_location is not None:
        settings['current_location'] = current_location
    return settings
