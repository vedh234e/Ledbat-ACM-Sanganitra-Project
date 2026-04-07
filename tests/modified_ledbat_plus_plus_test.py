import subprocess
import sys
import re

NS3_CMD = ["./ns3", "run", "scratch/ModifiedLedbatPlusPlusScratch.cc"]

# Match queue delay + effective target 
SS_RE = re.compile(r"Queue delay\s*:\s*(\d+).*?effective\s*=\s*(\d+)", re.IGNORECASE)

# Match slow start exit 
EXIT_RE = re.compile(r"Exiting initial slow start", re.IGNORECASE)

proc = subprocess.run(
    NS3_CMD,
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True
)

log = proc.stdout.splitlines()

records = []
last_qd = None
last_td = None

for line in log:
    ss_match = SS_RE.search(line)
    if ss_match:
        last_qd = int(ss_match.group(1))
        last_td = int(ss_match.group(2))

    if EXIT_RE.search(line):
        if last_qd is not None and last_td is not None:
            records.append((last_qd, last_td))
        last_qd = None
        last_td = None


# ---- OUTPUT ----

if not records:
    print("FAIL: No slow start exit detected")
    sys.exit(1)

all_pass = True

for i, (qd, td) in enumerate(records):
    threshold = 0.75 * td

    print(f"\n--- Slow Start Exit {i} ---")
    print(f"Queue delay        : {qd}")
    print(f"Effective target   : {td}")
    print(f"Exit threshold     : {threshold}")

    if qd > threshold:
        print("Result             : FAIL (Exited too late)")
        all_pass = False
    else:
        print("Result             : PASS (Early exit correct)")

if all_pass:
    print("\nPASS: LEDBAT++ slow start exit behavior is correct")
    sys.exit(0)
else:
    print("\nFAIL: Slow start exit condition violated")
    sys.exit(1)