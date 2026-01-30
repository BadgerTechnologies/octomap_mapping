{% from 'badger-lib.sls' import enable_service with context %}
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

enable_costmaps_3d_yaml:
  file.managed:
    - makedirs: true
    - name: {{ config_dir }}/octomap_server/enable_costmaps_3d.yaml
    - source: salt://truck/config/octomap_server_launch/enable_costmaps_3d.py
    - template: py

{{ enable_service('occupancy-mapping-3d', ['octomap_server_sensor_defaults_yaml', 'octomap_server_sensor_defaults_odom_yaml', 'octomap_server_sensor_defaults_map_yaml', 'octomap_servers_yaml']) }}
