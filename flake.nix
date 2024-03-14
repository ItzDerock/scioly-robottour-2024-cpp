{
  description = "Flake template for Arduino projects";

  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    nixpkgs.url = "github:NixOS/nixpkgs";
  };

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachSystem [ "x86_64-linux" ] (system:
      let
        name = "scioly";

        pkgs = import nixpkgs {
          inherit system;
        };

        python = pkgs.python3;

        pythonWithExtras = python.buildEnv.override {
          extraLibs = [ ];
        };

        # The variables starting with underscores are custom,
        # the ones starting with ARDUINO are used by arduino-cli.
        # See https://arduino.github.io/arduino-cli/0.35/configuration/ .

        # Store everything that arduino-cli downloads in a directory
        # reserved for this project, and following the XDG specification,
        # if the variable is available.

        # The _ARDUINO_PYTHON3 variable is passed to arduino-cli via the Makefile.
        arduinoShellHookPaths = ''
          if [ -z "''${_ARDUINO_PROJECT_DIR:-}" ]; then
            if [ -n "''${_ARDUINO_ROOT_DIR:-}" ]; then
              export _ARDUINO_PROJECT_DIR="''${_ARDUINO_ROOT_DIR}/${name}"
            elif [ -n "''${XDG_CACHE_HOME:-}" ]; then
              export _ARDUINO_PROJECT_DIR="''${XDG_CACHE_HOME}/arduino/${name}"
            else
              export _ARDUINO_PROJECT_DIR="''${HOME}/.arduino/${name}"
            fi
          fi
          # The variables below are respected by arduino-cli
          export ARDUINO_DIRECTORIES_USER=$_ARDUINO_PROJECT_DIR
          export ARDUINO_DIRECTORIES_DATA=$_ARDUINO_PROJECT_DIR
          export ARDUINO_DIRECTORIES_DOWNLOADS=$_ARDUINO_PROJECT_DIR/staging
          # This is used to override Python when invoking the Makefile
          export _ARDUINO_PYTHON3=${python}
        '';

        devShellArduinoCLI = pkgs.mkShell {
          name = "${name}-dev";
          packages = with pkgs; [
            arduino-cli       # For compiling and uploading the sketch
            git               # For embedding a version hash into the sketch
            gnumake           # To provide somewhat standardized commands to compile, upload, and monitor the sketch
            picocom           # To monitor the serial output
            pythonWithExtras  # So that the python3 wrapper of the esp8266 downloaded code can find a working python interpreter on the path
            stdenv.cc.cc.lib
            systemdLibs 
          ];
          # LD_LIBRARY_PATH = "${pkgs.stdenv.cc.cc.lib}/lib";
          LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath [
            pkgs.stdenv.cc.cc
            pkgs.systemdLibs.out
            # You can use the nix-index package to locate them, e.g. nix-locate -w --top-level --at-root /lib/libudev.so.1
          ];
          shellHook = ''
            ${arduinoShellHookPaths}
          #  echo "==> Using arduino-cli version $(arduino-cli version)"
          #  echo "    Storing arduino-cli data for this project in '$_ARDUINO_PROJECT_DIR'"
          '';
        };

      in
        rec {
          devShells = {
            inherit
              devShellArduinoCLI
            ;
          };

          # Development shell spawned by `nix develop`
          devShells.default = devShellArduinoCLI;
        }
    );
}
