#!/bin/bash

FLASK_DIR=$( cd -- "$(dirname "${0}")" >/dev/null 2>&1 ; pwd -P )

cd "$FLASK_DIR/../src/flask/"

python3 app.py
