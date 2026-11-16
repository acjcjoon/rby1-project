"""Compose the opt-in nvblox experiment without changing station localization."""

from copy import deepcopy


def configure_navigation(settings, *, use_scan, use_nvblox, occupancy_map='',
                         nvblox_settings=None):
    """Return independent settings; explicit sensor layers never imply free unknown space."""
    settings = deepcopy(settings)
    if use_nvblox:
        if not nvblox_settings:
            raise ValueError('nvblox navigation settings are required')
        # Replace individual ROS parameter values, including the entire controller
        # plugin dictionary, so omni-only parameters do not leak into RPP.
        for name, node in nvblox_settings.items():
            if name in ('local_costmap', 'global_costmap'):
                settings[name][name]['ros__parameters'].update(
                    deepcopy(node[name]['ros__parameters']))
            else:
                settings[name]['ros__parameters'].update(deepcopy(node['ros__parameters']))
    for name in ('local_costmap', 'global_costmap'):
        costmap = settings[name][name]['ros__parameters']
        plugins = costmap['plugins']
        if occupancy_map and name == 'global_costmap':
            costmap['rolling_window'] = False
            plugins = ['static_layer'] + [plugin for plugin in plugins if plugin != 'static_layer']
            costmap['static_layer'] = {
                'plugin': 'nav2_costmap_2d::StaticLayer',
                'map_subscribe_transient_local': True,
                'map_topic': '/rby1/vslam/nav2/map',
            }
        if not use_scan:
            plugins = [plugin for plugin in plugins if plugin != 'obstacle_layer']
            costmap.pop('obstacle_layer', None)
            if not use_nvblox and (name == 'local_costmap' or not occupancy_map):
                costmap['track_unknown_space'] = False
        if use_nvblox:
            # Inflation must see the newly marked nvblox obstacle cells.
            plugins = [plugin for plugin in plugins if plugin != 'nvblox_layer']
            index = plugins.index('inflation_layer') if 'inflation_layer' in plugins else len(plugins)
            plugins.insert(index, 'nvblox_layer')
            costmap['track_unknown_space'] = True
        costmap['plugins'] = plugins
    return settings
