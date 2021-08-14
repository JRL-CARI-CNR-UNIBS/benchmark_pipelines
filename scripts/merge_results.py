#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

author: jacobi
"""

import yaml

load_from_parameter_server=True

with open('../config/DOF6BENCHMARK1.yaml') as f:
    param1 = yaml.safe_load(f)
with open('../config/DOF6_msdirrt_comparison.yaml') as f:
    param2 = yaml.safe_load(f)

param1.update(param2)