{
  description = "FC6-II rocket profiles - parallel Nix build pipeline";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-24.11";

  outputs = { self, nixpkgs }:
    let
      systems = [ "x86_64-linux" "aarch64-linux" ];
      forAll = f: builtins.listToAttrs (
        map (s: { name = s; value = f s; }) systems
      );
      profile = "${self}/rockets/s-lx";
      ciScript = pkgs: name: extra: pkgs.writeShellApplication {
        inherit name;
        runtimeInputs = with pkgs; [ gnumake gcc python3 ];
        text = ''
          work="$(mktemp -d)"
          mkdir -p "$work/rockets"
          cp -r ${profile} "$work/rockets/s-lx"
          cp -r ${self}/shared "$work/shared"
          chmod -R u+w "$work"
          exec make -C "$work/rockets/s-lx" ${extra} PYTHON=${pkgs.python3}/bin/python3
        '';
      };
    in {
      devShells = forAll (s:
        let pkgs = import nixpkgs { system = s; }; in {
          default = pkgs.mkShell {
            name = "fc6-rockets";
            packages = with pkgs; [
              gcc
              gnumake
              pkg-config
              python3
              cargo
              rustc
              verilator
              zlib
              xorg.libX11
            ];
          };
        }
      );

      packages = forAll (s:
        let
          pkgs = import nixpkgs { system = s; };
        in {
          slx-ci = ciScript pkgs "slx-ci" "ci";
          slx-ci-live = ciScript pkgs "slx-ci-live" "ci-live";
          slx-vis = pkgs.rustPlatform.buildRustPackage {
            pname = "rocket-vis";
            version = "0.1.0";
            src = ./shared/vis/rust;
            cargoLock.lockFile = ./shared/vis/rust/Cargo.lock;
            buildInputs = with pkgs; [ xorg.libX11 ];
          };
        }
      );

      apps = forAll (s:
        let pkgs = import nixpkgs { system = s; }; in {
          slx-ci = {
            type = "app";
            program = "${self.packages.${s}.slx-ci}/bin/slx-ci";
          };
          slx-ci-live = {
            type = "app";
            program = "${self.packages.${s}.slx-ci-live}/bin/slx-ci-live";
          };
          slx-vis = {
            type = "app";
            program = "${pkgs.writeShellApplication {
              name = "slx-vis";
              text = ''
                cd ${profile}
                exec ${self.packages.${s}.slx-vis}/bin/rocket-vis --config config/vis.json "$@"
              '';
            }}/bin/slx-vis";
          };
        }
      );
    };
}
