# Forwards to the master repo's build.ps1, which is the single definition of the
# ESP-IDF environment for all three projects.
#
#   .\build.ps1 build
#
# Firmware is delivered over OTA; do not flash over serial from this script.
#
# WHY IT ONLY FORWARDS: this script used to set the environment itself, and it
# pointed at export.ps1's interpreter (C:\Espressif\tools\python_env\
# idf6.0_py3.11_env) instead of the VS Code extension's
# (C:\Espressif\tools\python\v6.0.1\venv). A build directory is pinned to
# whichever ran first, so mixing them fails with
#   "'...idf6.0_py3.11_env\python.exe' is currently active while the project was
#    configured with '...tools\python\v6.0.1\venv\python.exe'. Run 'idf.py fullclean'"
# One copy of the environment cannot drift out of sync with itself.
& "$PSScriptRoot\..\elotto\build.ps1" -C $PSScriptRoot @args
