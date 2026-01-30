#!/usr/bin/env python3
import yaml

from badger_salt_lib.template_helpers import Pillar


def run():
    global __pillar__
    pillar = Pillar(__pillar__)
    enable_costmaps_3d_params = {}

    # We don't need to check enable_local_costmap_3d currently
    # but it could be added in the future if needed
    enable_global_costmap_3d = pillar.get(
        "configuration:software:costmaps:enable_global_costmap_3d", default=False
    )

    enable_costmaps_3d_params["enable_global_costmap_3d"] = enable_global_costmap_3d

    return yaml.dump(enable_costmaps_3d_params)
