#!/usr/bin/env bash

set -ex

MODEL_PATH="/tmp/model.pb"
python3 -m robomaker.inference_worker ${MODEL_PATH}
