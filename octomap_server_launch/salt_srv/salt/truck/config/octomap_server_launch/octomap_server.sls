{% from 'badger-lib.sls' import enable_service, disable_service with context %}
{% set config_dir = salt['pillar.get']('configuration:config_dir') %}

octomap_server_sensor_defaults_yaml:
  file.managed:
    - makedirs: true
    - name: {{ config_dir }}/octomap_server/octomap_server_sensor_defaults.yaml
    - source: salt://truck/config/octomap_server_launch/octomap_server_sensor_default.j2
    - template: jinja

octomap_server_sensor_defaults_odom_yaml:
  file.managed:
    - makedirs: true
    - name: {{ config_dir }}/octomap_server/octomap_server_sensor_defaults_odom.yaml
    - source: salt://truck/config/octomap_server_launch/octomap_server_sensor_default_odom.j2
    - template: jinja

octomap_server_sensor_defaults_map_yaml:
  file.managed:
    - makedirs: true
    - name: {{ config_dir }}/octomap_server/octomap_server_sensor_defaults_map.yaml
    - source: salt://truck/config/octomap_server_launch/octomap_server_sensor_default_map.j2
    - template: jinja

octomap_servers_yaml:
  file.managed:
    - makedirs: true
    - name: {{ config_dir }}/octomap_server/octomap_servers.yaml
    - source: salt://truck/config/octomap_server_launch/octomap_servers.j2
    - template: jinja

{% if salt['pillar.get']('configuration:software:costmaps:enable_local_costmap_3d', False) %}
{{ enable_service('octomap-server-odom-frame', ['octomap_server_sensor_defaults_yaml', 'octomap_server_sensor_defaults_odom_yaml', 'octomap_server_sensor_defaults_map_yaml', 'octomap_servers_yaml']) }}
{% else %}
{{ disable_service('octomap-server-odom-frame') }}
{% endif %}

{% if salt['pillar.get']('configuration:software:costmaps:enable_global_costmap_3d', False) %}
{{ enable_service('octomap-server-map-frame', ['octomap_server_sensor_defaults_yaml', 'octomap_server_sensor_defaults_odom_yaml', 'octomap_server_sensor_defaults_map_yaml', 'octomap_servers_yaml']) }}
{% else %}
{{ disable_service('octomap-server-map-frame') }}
{% endif %}
