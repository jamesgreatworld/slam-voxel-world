#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""agent_demo.py — 模拟 agent 的链式查询:上一步结果决定下一步查什么。"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mcp_client import Client


def show(step, tool, args, res):
    print("\n" + "=" * 74)
    print("[%s] agent 调用 %s(%s)" % (step, tool, json.dumps(args, ensure_ascii=False)))
    print("-" * 74)
    print(json.dumps(res, ensure_ascii=False, indent=2)[:1400])


c = Client()

# 1. 先摸清场景规模,顺便拿到"叠放物体"线索
s = c.call("scene_summary", {})
show("1", "scene_summary", {}, s)

# 2. 机器人在哪(位置 -> 房间)
p = c.call("get_robot_pose", {})
show("2", "get_robot_pose", {}, p)
r = c.call("get_robot_room", {})
show("3", "get_robot_room", {}, r)

# 3. 用上一步得到的房间 id,查这个房间里有什么
rid = r.get("room_id")
if rid:
    show("4", "room_contents", {"room_id": rid}, c.call("room_contents", {"room_id": rid}))

# 4. 用 scene_summary 里的叠放线索,验证父子链(物体->支撑物体->房间->building)
stacked = s.get("objects_on_other_objects") or []
if stacked:
    oid = stacked[0]["id"]
    show("5", "get_object", {"object_id": oid}, c.call("get_object", {"object_id": oid}))
    show("6", "get_relations", {"node_id": oid}, c.call("get_relations", {"node_id": oid}))

# 5. 按名字找物体(取房间里出现过的一个名字)
name = None
rc = c.call("room_contents", {"room_id": rid}) if rid else {}
for o in (rc.get("objects") or []):
    if o.get("name"):
        name = o["name"]
        break
if name:
    show("7", "find_object", {"name": name}, c.call("find_object", {"name": name}))

# 6. 错误处理
show("8", "room_contents", {"room_id": "room:999"},
     c.call("room_contents", {"room_id": "room:999"}))
c.close()
