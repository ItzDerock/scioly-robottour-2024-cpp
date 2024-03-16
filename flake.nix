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
    in
    {
      devShell.x86_64-linux = pkgs.mkShell {
        buildInputs = with pkgs; [
          cmake
          gcc-arm-embedded
          pico-sdk-sub
          python3
          glibc_multi # headers
          microcom # serial 
          clang-tools # lsp
        ];

        PICO_SDK_PATH = "${pico-sdk-sub}/lib/pico-sdk";
        # PICO_SDK_PATH = builtins.toString ./pico-sdk;

        shellHook = ''
          git pull
        '';
      };
    };
}
