# #------------------With voice command V.2 (SingleThreadedExecutor)------------------#

# #!/usr/bin/env python3
# """LLM-driven Panda control (path b: import primitives, one shared MoveItPy context).

# Two input paths, one dispatch:
#   - stdin REPL ("cmd> ")           -- interactive, main thread
#   - /voice_command (std_msgs/String) -- publish with `ros2 topic pub`, sets up a
#     future speech front-end; runs in a background spin thread.

# Each instruction: text -> Ollama (schema-constrained JSON) -> parse -> clamp (pose)
# -> dispatch. A lock serializes dispatch so the REPL and the topic never drive the
# single MoveItPy context concurrently.

# Usage:
#   ros2 run twin_moveit_scripts llm_move
#   # REPL: type "go to the ready position" or "move to x 0.3 y -0.2 z 0.5"; Ctrl-D/"quit" to exit.
#   # Topic: ros2 topic pub --once /voice_command std_msgs/String "data: 'go home'"
# """
# import json
# import threading
# import urllib.request

# import rclpy
# from rclpy.node import Node
# from rclpy.executors import SingleThreadedExecutor
# from std_msgs.msg import String

# from twin_moveit_scripts.move_to import (
#     setup, shutdown, move_to_named, move_to_pose,
# )

# OLLAMA_URL = "http://localhost:11434/api/chat"
# MODEL = "qwen2.5:3b-instruct"

# # Conservative reachable box for the Panda in panda_link0 frame (meters).
# BOUNDS = {"x": (0.15, 0.65), "y": (-0.5, 0.5), "z": (0.15, 0.9)}

# SYSTEM_PROMPT = (
#     "You translate a natural-language instruction for a Panda robot arm into ONE JSON tool call. "
#     "Two tools:\n"
#     "- move_to_named: go to a preset joint pose. args: {\"name\": \"ready\"|\"home\"}. "
#     "Use for words like ready, home, reset.\n"
#     "- move_to_pose: move the gripper to a Cartesian point. args: {\"x\": float, \"y\": float, \"z\": float} in meters.\n"
#     "Pick move_to_named when the user names a preset pose; pick move_to_pose only when explicit "
#     "x/y/z coordinates are given. Output JSON only."
# )

# SCHEMA = {
#     "oneOf": [
#         {"type": "object", "properties": {
#             "action": {"const": "move_to_named"},
#             "args": {"type": "object", "properties": {
#                 "name": {"type": "string", "enum": ["ready", "home"]}},
#                 "required": ["name"], "additionalProperties": False}},
#             "required": ["action", "args"], "additionalProperties": False},
#         {"type": "object", "properties": {
#             "action": {"const": "move_to_pose"},
#             "args": {"type": "object", "properties": {
#                 "x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"}},
#                 "required": ["x", "y", "z"], "additionalProperties": False}},
#             "required": ["action", "args"], "additionalProperties": False},
#     ]
# }


# def query_llm(text):
#     """Send one instruction, return parsed {action, args} dict (schema-guaranteed shape)."""
#     payload = {
#         "model": MODEL,
#         "stream": False,
#         "messages": [
#             {"role": "system", "content": SYSTEM_PROMPT},
#             {"role": "user", "content": text},
#         ],
#         "format": SCHEMA,
#         "options": {"temperature": 0},
#     }
#     req = urllib.request.Request(
#         OLLAMA_URL,
#         data=json.dumps(payload).encode(),
#         headers={"Content-Type": "application/json"},
#     )
#     with urllib.request.urlopen(req, timeout=30) as resp:
#         body = json.loads(resp.read().decode())
#     return json.loads(body["message"]["content"])


# def clamp_pose(args, logger):
#     """Snap x/y/z into the reachable box; log any change."""
#     out = {}
#     for k in ("x", "y", "z"):
#         lo, hi = BOUNDS[k]
#         v = float(args[k])
#         c = max(lo, min(hi, v))
#         if c != v:
#             logger.warn(f"clamped {k}: {v} -> {c} (bounds {lo}..{hi})")
#         out[k] = c
#     return out


# # ---- one dispatch path, shared by REPL and topic ----
# def handle_instruction(text, robot, arm, logger, params, twin, lock, source="repl"):
#     """query -> parse -> clamp (pose) -> dispatch. Serialized via lock so the two
#     input paths never drive the single MoveItPy context concurrently.

#     Returns the MoveResult from the primitive (or None if no motion ran), so
#     callers/future task-level logic can react to plan-fail / final-pose."""
#     text = text.strip()
#     if not text:
#         return

#     try:
#         call = query_llm(text)
#     except Exception as e:
#         logger.error(f"[{source}] LLM query failed: {e}")
#         return

#     action = call.get("action")
#     args = call.get("args", {})
#     logger.info(f"[{source}] LLM -> {action} {args}")

#     # Serialize actual arm motion; a second instruction waits its turn.
#     # Primitives now return a MoveResult (ok / stage / final_pose / error);
#     # report it back so later self-correction / task planning can react to it.
#     with lock:
#         if action == "move_to_named":
#             result = move_to_named(robot, arm, logger, params, args["name"])
#         elif action == "move_to_pose":
#             safe = clamp_pose(args, logger)
#             result = move_to_pose(robot, arm, logger, params,
#                                   safe["x"], safe["y"], safe["z"])
#         else:
#             logger.error(f"[{source}] unknown action: {action}")
#             return None

#     if result.ok:
#         logger.info(f"[{source}] DONE: {result}")
#     else:
#         logger.error(f"[{source}] MOVE FAILED: {result}")
#     return result


# class VoiceCommandNode(Node):
#     """Subscribes to /voice_command and routes each String through handle_instruction."""

#     def __init__(self, robot, arm, logger, params, twin, lock):
#         super().__init__("llm_move_sub")
#         self._robot = robot
#         self._arm = arm
#         self._logger = logger
#         self._params = params
#         self._twin = twin
#         self._lock = lock
#         self.create_subscription(String, "/voice_command", self._on_msg, 10)
#         logger.info("Subscribed to /voice_command (std_msgs/String).")

#     def _on_msg(self, msg):
#         handle_instruction(
#             msg.data, self._robot, self._arm, self._logger, self._params,
#             self._twin, self._lock, source="topic",
#         )


# def main():
#     robot, arm, logger, params, twin = setup(node_name="llm_move")

#     lock = threading.Lock()

#     # Topic path: its own node, spun in a background thread so callbacks fire
#     # while the REPL blocks on input() in the main thread.
#     spin_thread = None
#     try:
#         sub_node = VoiceCommandNode(robot, arm, logger, params, twin, lock)
#         executor = SingleThreadedExecutor()
#         executor.add_node(sub_node)

#         def _spin():
#             try:
#                 executor.spin()
#             except Exception as e:  # otherwise a thread death is silent
#                 logger.error(f"spin thread died: {e}")

#         spin_thread = threading.Thread(target=_spin, daemon=True)
#         spin_thread.start()
#         logger.info("Topic path started (spin thread live).")
#     except Exception as e:
#         logger.error(f"could not start /voice_command path: {e!r} -- REPL still works")
#         executor = None
#         sub_node = None

#     logger.info("LLM control ready. Type an instruction (Ctrl-D or 'quit' to exit). "
#                 "Also listening on /voice_command.")

#     try:
#         while True:
#             try:
#                 text = input("cmd> ")
#             except EOFError:
#                 break
#             if text.strip().lower() in ("quit", "exit"):
#                 break
#             handle_instruction(text, robot, arm, logger, params, twin, lock, source="repl")
#     finally:
#         logger.info("Shutting down.")
#         if executor is not None:
#             executor.shutdown()
#         if sub_node is not None:
#             sub_node.destroy_node()
#         shutdown(twin)


# if __name__ == "__main__":
#     main()

#------------------With voice command V.2 (SingleThreadedExecutor)------------------#

#------------------With voice command V.2 (SingleThreadedExecutor)------------------#

#!/usr/bin/env python3
"""LLM-driven Panda control (path b: import primitives, one shared MoveItPy context).

Two input paths, one dispatch:
  - stdin REPL ("cmd> ")           -- interactive, main thread
  - /voice_command (std_msgs/String) -- publish with `ros2 topic pub`, sets up a
    future speech front-end; runs in a background spin thread.

Each instruction: text -> Ollama (schema-constrained JSON) -> parse -> clamp (pose)
-> dispatch. A lock serializes dispatch so the REPL and the topic never drive the
single MoveItPy context concurrently.

Usage:
  ros2 run twin_moveit_scripts llm_move
  # REPL: type "go to the ready position" or "move to x 0.3 y -0.2 z 0.5"; Ctrl-D/"quit" to exit.
  # Topic: ros2 topic pub --once /voice_command std_msgs/String "data: 'go home'"
"""
import json
import threading
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

from twin_moveit_scripts.move_to import (
    setup, shutdown, move_to_named, move_to_pose, pick, place, pick_and_place,
)

OLLAMA_URL = "http://localhost:11434/api/chat"
MODEL = "qwen2.5:3b-instruct"

# Conservative reachable box for the Panda in panda_link0 frame (meters).
BOUNDS = {"x": (0.15, 0.65), "y": (-0.5, 0.5), "z": (0.15, 0.9)}

SYSTEM_PROMPT = (
    "You translate a natural-language instruction for a Panda robot arm into ONE JSON tool call. "
    "Tools:\n"
    "- move_to_named: go to a preset joint pose. args: {\"name\": \"ready\"|\"home\"}. "
    "Use for words like ready, home, reset.\n"
    "- move_to_pose: move the gripper to a Cartesian point. args: {\"x\": float, \"y\": float, \"z\": float} in meters.\n"
    "- pick: grasp a known object. args: {\"object\": \"box\"}. Use for words like pick, grab, grasp, pick up.\n"
    "- place: put the held object down at a known target. args: {\"target\": \"table\"}. "
    "Use for words like place, put, drop, put down, set down.\n"
    "- pick_and_place: grasp an object then place it at a target in one motion. "
    "args: {\"object\": \"box\", \"target\": \"table\"}. Use when the instruction names BOTH a thing "
    "to move and where to move it, e.g. 'move the box to the table' or 'put the box on the table'.\n"
    "Pick move_to_named when the user names a preset pose; move_to_pose only when explicit "
    "x/y/z coordinates are given; pick/place/pick_and_place for manipulation of named objects. "
    "Output JSON only."
)

SCHEMA = {
    "oneOf": [
        {"type": "object", "properties": {
            "action": {"const": "move_to_named"},
            "args": {"type": "object", "properties": {
                "name": {"type": "string", "enum": ["ready", "home"]}},
                "required": ["name"], "additionalProperties": False}},
            "required": ["action", "args"], "additionalProperties": False},
        {"type": "object", "properties": {
            "action": {"const": "move_to_pose"},
            "args": {"type": "object", "properties": {
                "x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"}},
                "required": ["x", "y", "z"], "additionalProperties": False}},
            "required": ["action", "args"], "additionalProperties": False},
        {"type": "object", "properties": {
            "action": {"const": "pick"},
            "args": {"type": "object", "properties": {
                "object": {"type": "string", "enum": ["box"]}},
                "required": ["object"], "additionalProperties": False}},
            "required": ["action", "args"], "additionalProperties": False},
        {"type": "object", "properties": {
            "action": {"const": "place"},
            "args": {"type": "object", "properties": {
                "target": {"type": "string", "enum": ["table"]}},
                "required": ["target"], "additionalProperties": False}},
            "required": ["action", "args"], "additionalProperties": False},
        {"type": "object", "properties": {
            "action": {"const": "pick_and_place"},
            "args": {"type": "object", "properties": {
                "object": {"type": "string", "enum": ["box"]},
                "target": {"type": "string", "enum": ["table"]}},
                "required": ["object", "target"], "additionalProperties": False}},
            "required": ["action", "args"], "additionalProperties": False},
    ]
}


def query_llm(text):
    """Send one instruction, return parsed {action, args} dict (schema-guaranteed shape)."""
    payload = {
        "model": MODEL,
        "stream": False,
        "messages": [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": text},
        ],
        "format": SCHEMA,
        "options": {"temperature": 0},
    }
    req = urllib.request.Request(
        OLLAMA_URL,
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(req, timeout=30) as resp:
        body = json.loads(resp.read().decode())
    return json.loads(body["message"]["content"])


def clamp_pose(args, logger):
    """Snap x/y/z into the reachable box; log any change."""
    out = {}
    for k in ("x", "y", "z"):
        lo, hi = BOUNDS[k]
        v = float(args[k])
        c = max(lo, min(hi, v))
        if c != v:
            logger.warn(f"clamped {k}: {v} -> {c} (bounds {lo}..{hi})")
        out[k] = c
    return out


# ---- one dispatch path, shared by REPL and topic ----
def handle_instruction(text, robot, arm, logger, params, twin, lock, source="repl"):
    """query -> parse -> clamp (pose) -> dispatch. Serialized via lock so the two
    input paths never drive the single MoveItPy context concurrently.

    Returns the MoveResult from the primitive (or None if no motion ran), so
    callers/future task-level logic can react to plan-fail / final-pose."""
    text = text.strip()
    if not text:
        return

    try:
        call = query_llm(text)
    except Exception as e:
        logger.error(f"[{source}] LLM query failed: {e}")
        return

    action = call.get("action")
    args = call.get("args", {})
    logger.info(f"[{source}] LLM -> {action} {args}")

    # Serialize actual arm motion; a second instruction waits its turn.
    # Primitives now return a MoveResult (ok / stage / final_pose / error);
    # report it back so later self-correction / task planning can react to it.
    with lock:
        if action == "move_to_named":
            result = move_to_named(robot, arm, logger, params, args["name"])
        elif action == "move_to_pose":
            safe = clamp_pose(args, logger)
            result = move_to_pose(robot, arm, logger, params,
                                  safe["x"], safe["y"], safe["z"])
        elif action == "pick":
            result = pick(robot, arm, logger, params, twin, args["object"])
        elif action == "place":
            result = place(robot, arm, logger, params, twin, args["target"])
        elif action == "pick_and_place":
            # pick_and_place returns a (pick_result, place_result) tuple, not a
            # single MoveResult. Log both stages, then hand the shared tail below
            # ONE result to report on: the place if we got that far, else the
            # pick that stopped the cycle.
            pick_result, place_result = pick_and_place(
                robot, arm, logger, params, twin, args["object"], args["target"])
            logger.info(f"[{source}] pick_and_place: "
                        f"pick={pick_result.ok}, "
                        f"place={place_result.ok if place_result else 'N/A (pick failed)'}")
            result = place_result if place_result is not None else pick_result
        else:
            logger.error(f"[{source}] unknown action: {action}")
            return None

    if result.ok:
        logger.info(f"[{source}] DONE: {result}")
    else:
        logger.error(f"[{source}] MOVE FAILED: {result}")
    return result


class VoiceCommandNode(Node):
    """Subscribes to /voice_command and routes each String through handle_instruction."""

    def __init__(self, robot, arm, logger, params, twin, lock):
        super().__init__("llm_move_sub")
        self._robot = robot
        self._arm = arm
        self._logger = logger
        self._params = params
        self._twin = twin
        self._lock = lock
        self.create_subscription(String, "/voice_command", self._on_msg, 10)
        logger.info("Subscribed to /voice_command (std_msgs/String).")

    def _on_msg(self, msg):
        handle_instruction(
            msg.data, self._robot, self._arm, self._logger, self._params,
            self._twin, self._lock, source="topic",
        )


def main():
    robot, arm, logger, params, twin = setup(node_name="llm_move")

    lock = threading.Lock()

    # Topic path: its own node, spun in a background thread so callbacks fire
    # while the REPL blocks on input() in the main thread.
    spin_thread = None
    try:
        sub_node = VoiceCommandNode(robot, arm, logger, params, twin, lock)
        executor = SingleThreadedExecutor()
        executor.add_node(sub_node)

        def _spin():
            try:
                executor.spin()
            except Exception as e:  # otherwise a thread death is silent
                logger.error(f"spin thread died: {e}")

        spin_thread = threading.Thread(target=_spin, daemon=True)
        spin_thread.start()
        logger.info("Topic path started (spin thread live).")
    except Exception as e:
        logger.error(f"could not start /voice_command path: {e!r} -- REPL still works")
        executor = None
        sub_node = None

    logger.info("LLM control ready. Type an instruction (Ctrl-D or 'quit' to exit). "
                "Also listening on /voice_command.")

    try:
        while True:
            try:
                text = input("cmd> ")
            except EOFError:
                break
            if text.strip().lower() in ("quit", "exit"):
                break
            handle_instruction(text, robot, arm, logger, params, twin, lock, source="repl")
    finally:
        logger.info("Shutting down.")
        if executor is not None:
            executor.shutdown()
        if sub_node is not None:
            sub_node.destroy_node()
        shutdown(twin)


if __name__ == "__main__":
    main()