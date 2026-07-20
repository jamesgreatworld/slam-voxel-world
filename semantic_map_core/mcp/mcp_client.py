#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mcp_client.py — 极简 MCP client(agent 侧)。spawn server, 按顺序发 tools/call。

用法:
  python3 mcp_client.py list
  python3 mcp_client.py call get_robot_room
  python3 mcp_client.py call find_object '{"name":"cabinets"}'
  python3 mcp_client.py script calls.json     # 批量: [["tool",{args}], ...]
"""
import json
import os
import subprocess
import sys

SERVER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "smc_mcp_server.py")


class Client:
    def __init__(self, extra_args=()):
        self.p = subprocess.Popen([sys.executable, SERVER, *extra_args],
                                  stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  text=True, bufsize=1)
        self.i = 0
        self._rpc("initialize", {"protocolVersion": "2024-11-05",
                                 "capabilities": {},
                                 "clientInfo": {"name": "agent", "version": "0"}})

    def _rpc(self, method, params=None):
        self.i += 1
        req = {"jsonrpc": "2.0", "id": self.i, "method": method}
        if params is not None:
            req["params"] = params
        self.p.stdin.write(json.dumps(req) + "\n")
        self.p.stdin.flush()
        return json.loads(self.p.stdout.readline())

    def list_tools(self):
        return self._rpc("tools/list")["result"]["tools"]

    def call(self, name, args=None):
        r = self._rpc("tools/call", {"name": name, "arguments": args or {}})
        if "error" in r:
            return {"_error": r["error"]}
        return json.loads(r["result"]["content"][0]["text"])

    def close(self):
        try:
            self.p.stdin.close(); self.p.wait(timeout=3)
        except Exception:
            self.p.kill()


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "list"
    c = Client()
    if mode == "list":
        for t in c.list_tools():
            print("- %-16s %s" % (t["name"], t["description"]))
    elif mode == "call":
        args = json.loads(sys.argv[3]) if len(sys.argv) > 3 else {}
        print(json.dumps(c.call(sys.argv[2], args), ensure_ascii=False, indent=2))
    elif mode == "script":
        calls = json.load(open(sys.argv[2]))
        for name, args in calls:
            print("\n" + "=" * 72)
            print(">>> agent 调用: %s(%s)" % (name, json.dumps(args, ensure_ascii=False)))
            print("-" * 72)
            print(json.dumps(c.call(name, args), ensure_ascii=False, indent=2))
    c.close()


if __name__ == "__main__":
    main()
