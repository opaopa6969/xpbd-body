#!/bin/sh
cd "$(dirname "$0")"
NODE_BIN=$(command -v node)
# Prefer Node 20+ (crypto global) if available via nvm
if [ -x "$HOME/.nvm/versions/node/v20.20.1/bin/node" ]; then
  NODE_BIN="$HOME/.nvm/versions/node/v20.20.1/bin/node"
fi
exec "$NODE_BIN" mcp/server.mjs
