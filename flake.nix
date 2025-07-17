{
  description = "A basic flake with a shell";
  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
  inputs.systems.url = "github:nix-systems/default";
  inputs.flake-utils = {
    url = "github:numtide/flake-utils";
    inputs.systems.follows = "systems";
  };

  outputs =
    { nixpkgs, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
      in
      {
        formatter = pkgs.nixfmt-tree;
        devShells.default = pkgs.mkShell rec {
          packages = [
            pkgs.bashInteractive
          ];
          buildInputs = [
            pkgs.libGL
            pkgs.xorg.libX11
            pkgs.xorg.libXcursor
            pkgs.xorg.libXrandr
            pkgs.xorg.libXi
          ];
          env = {
            LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath buildInputs;
            RUST_BACKTRACE = "full";
            # Current version of winit that kiss3d's dependency glutin is using looks not support latest wayland
            # https://github.com/sebcrozet/kiss3d/issues/336
            # This issue looks resolved in winit 0.28.7 but glutin used 0.27 in 0.29.0 and 0.3x uses raw_windows-handle instead
            # kiss3d looks stale, so we should use xwayland
            WINIT_UNIX_BACKEND = "x11";
          };
        };
      }
    );
}
