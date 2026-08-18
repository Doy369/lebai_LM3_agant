# Configuration policy

Only non-sensitive examples belong in this directory. Robot addresses, Web control
tokens, model API keys, and device-specific production calibration must be supplied
through environment variables or ignored local files.

- `camera/`: public camera configuration examples;
- `*.local.ini`: ignored machine-specific overrides.

Runtime data continues to use the paths selected by the command line or environment.
