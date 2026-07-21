# isaac_bundle — cenas prontas para Isaac Sim (versionadas)

Ficheiros **auto-contidos** (o `.usd` embute mesh + gaussianos) → git-friendly, sem `.ply` pesados.
Os `.ply` de gaussianos (para SuperSplat) ficam fora do git; transfere-os por scp se precisares.

## Cenas
- `d435i_3pass/` — captura D435i de 3 passadas (h/cima/baixo). PSNR 27.53. Mesh TSDF métrico.
  - `d435i_3pass_isaac_sim.usd` — **abre → Play** (mesh colidível + física + cubo de teste).
  - `d435i_3pass_isaac.usd` — base (mesh + gaussianos como pontos). O sim referencia-o.
  - `d435i_3pass_isaac.usdz` — versão zipada (Quick Look no Mac).
  - `run_check.py` / `run_render.py` / `run_replicator_dataset.py` — scripts headless (Isaac 5.0).
  - `HANDOFF_CLAUDE_2060.md` — prompt/instruções para a 2060.
  - `mesh_preview.png` — preview do mesh.

## Uso na 2060 (resumo; detalhe no HANDOFF)
```
docker run ... -v ~/<caminho>/isaac_bundle/d435i_3pass:/root/dtvf_isaac ... isaac-sim:5.0.0
./python.sh /root/dtvf_isaac/run_check.py  d435i_3pass_isaac_sim.usd
./python.sh /root/dtvf_isaac/run_render.py d435i_3pass_isaac_sim.usd
```
