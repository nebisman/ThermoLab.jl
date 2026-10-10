#!/usr/bin/env bash
# Reconstruye la documentación local de ThermoLab.jl (docs/build/) a partir del
# código y los docstrings actuales, sin desplegarla a GitHub Pages (el
# despliegue real lo hace el workflow .github/workflows/Documentation.yml al
# hacer push a main).
#
# Uso:
#   docs/build.sh            # actualiza el entorno de docs/ y reconstruye
#   docs/build.sh --serve    # además sirve el resultado en http://localhost:8000

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

echo "==> Actualizando el entorno de docs/ (Pkg.develop + instantiate)"
julia --project=docs -e '
    using Pkg
    Pkg.develop(PackageSpec(path=pwd()))
    Pkg.instantiate()
'

echo "==> Reconstruyendo la documentación (docs/make.jl)"
julia --project=docs docs/make.jl

echo "==> Documentación generada en docs/build/index.html"

if [[ "${1:-}" == "--serve" ]]; then
    echo "==> Sirviendo docs/build/ en http://localhost:8000 (Ctrl+C para detener)"
    cd docs/build
    python3 -m http.server 8000
fi
