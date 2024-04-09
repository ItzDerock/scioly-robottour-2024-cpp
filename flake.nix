{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs";
  };

  outputs = { self, nixpkgs }:
    let
      pkgs = import nixpkgs {
        system = "x86_64-linux";
      };

      pico-sdk-sub = with pkgs; (pico-sdk.overrideAttrs (o:
        rec {
          pname = "pico-sdk";
          version = "1.5.1";
          src = fetchFromGitHub {
            fetchSubmodules = true;
            owner = "raspberrypi";
            repo = pname;
            rev = version;
            sha256 = "sha256-GY5jjJzaENL3ftuU5KpEZAmEZgyFRtLwGVg3W1e/4Ho=";
          };
        }));

      fhs = pkgs.buildFHSUserEnv {
        name = "fhs-shell";
        targetPkgs = pkgs: with pkgs; [
          cmake
          gcc-arm-embedded
          pico-sdk-sub
          python3
          glibc_multi # headers
          minicom # serial 
          openocd-rp2040 # picotool
          clang-tools # lsp
        ];

        runScript = "bash";
        profile = ''
          export PICO_SDK_PATH="${pico-sdk-sub}/lib/pico-sdk"
        '';
      };
    in
    {
      devShells.x86_64-linux.default = fhs.env;
    };
}
