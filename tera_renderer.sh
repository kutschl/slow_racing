#!/bin/bash
mkdir ~/tera_renderer_build 
cd ~/tera_renderer_build 
wget https://github.com/acados/tera_renderer/archive/refs/tags/v0.0.35.zip 
unzip v0.0.35.zip 
# curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh && \
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
source $HOME/.cargo/env
cd tera_renderer-0.0.35/ 
cargo build --verbose --release 
cp target/release/t_renderer ~/acados/bin/t_renderer 
chmod +x ~/acados/bin/t_renderer
~/acados/bin/t_renderer '/home/alonso/acados/interfaces/acados_template/acados_template/c_templates_tera/**/*' 'main.in.c' '/home/ss24_racing1/acados_ocp.json' 'main_Spatial_Model.c'
