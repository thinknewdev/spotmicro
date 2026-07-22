#!/bin/bash
# SpotMicro local control panel — http://localhost:8080
exec python3 -m http.server 8080 --directory "$(dirname "$(readlink -f "$0")")"
