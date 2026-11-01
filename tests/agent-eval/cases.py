"""The cross-agent acceptance cases: five prompts against the committed fixture.

Each criterion is checked against one part of a transcript (see grade.py):

    "commands"  the shell commands the agent ran, one per line
    "answer"    the agent's final answer

kind "present" passes when `pattern` matches, "absent" when it does not, "first" when the
first `alog` subcommand the agent ran is `pattern`, and "manual" is printed for a person to
judge (some things a regex cannot decide). Patterns are case-insensitive. Where a figure
comes from the tool, tests/test_agent_eval.py checks that `alog` really prints it on the
fixture, so a change to the tool cannot leave an expectation here silently stale.
"""

FIXTURE = "largeprop-quad.bin"      # copied into the workspace by workspace.py
ULOG = "flight.ulg"                 # a PX4 ULog header, for the out-of-scope case

CASES = [
    {
        "id": "1-triage",
        "prompt": f"I flew my quad and this is the log: {FIXTURE}. Is it OK?",
        "criteria": [
            ("runs `alog info` first", "commands", "first", "info"),
            ("reports the integrity result", "answer", "present", r"integrity|intact|resync|no (integrity )?errors"),
            # the method string, or the same two events in words ("from 'not landed' to 'land complete'")
            ("names the window and its method", "answer", "present",
             r"NOT_LANDED\W*(->|→|to)\W*LAND_COMPLETE|not[ _-]?landed.{0,40}land[ _-]?complete"),
            ("says the log holds one flight", "answer", "present", r"\b(one|1|single)\b[^.\n]{0,20}\bflight"),
            ("surfaces a FAIL with its number", "answer", "present", r"58(\.\d+)?\s*(us|µs|microseconds)|12\.2\d*\s*%"),
            ("states numbers, not only verdicts", "answer", "manual",
             "each finding quotes its figure and threshold, not just 'fine' / 'bad'"),
        ],
    },
    {
        "id": "2-notch",
        "prompt": f"Check the harmonic notch on this log: {FIXTURE}",
        "criteria": [
            ("runs `alog info` first", "commands", "first", "info"),
            ("says the notch is disabled", "answer", "present", r"disabled|INS_HNTCH_ENABLE\W{0,4}0\b"),
            # \W{0,12}: "= 3", ": 3", "-> 3" or a table cell, "|     3 |"
            ("gives MODE 3 from the report", "answer", "present", r"INS_HNTCH_MODE\W{0,12}3\b"),
            # FREQ is just under the window's p01 fundamental: 55 on the default window, 60 on
            # --window rpm, which recipe Skill 4 uses (test_agent_eval checks both)
            ("gives FREQ from the report", "answer", "present", r"INS_HNTCH_FREQ\W{0,12}(55|60)\b"),
            ("names the flight that verifies it", "answer", "present", r"INS_LOG_BAT_MASK|batchfft"),
            ("does not call the notch working", "answer", "absent",
             r"notch (is |was )?(working|tracking)( well| correctly| fine)"),
            ("does not judge it by FTN1.PkAvg", "answer", "manual",
             "if FTN1.PkAvg is mentioned at all, it is not used as evidence of what the notch applied"),
        ],
    },
    {
        "id": "3-motors",
        "prompt": f"Are my motors balanced? Log: {FIXTURE}",
        "criteria": [
            ("runs `alog info` first", "commands", "first", "info"),
            ("uses the servo-to-motor map", "answer", "present", r"SERVO\w*_FUNCTION|channel map|M1\W{0,3}C3"),
            ("quotes the pitch trim", "answer", "present", r"58(\.\d+)?\s*(us|µs|microseconds)"),
            ("separates load from drag", "answer", "present",
             r"drive[- ]normali[sz]ed|RPM\s*(/|÷)\s*\(?duty|per unit of drive"),
            ("raises a breeze or the heading test", "answer", "present", r"breeze|wind|heading"),
            ("does not call the CG offset proven", "answer", "manual",
             "the CG figure is given as conditional: trim vs heading could not separate CG from a breeze"),
        ],
    },
    {
        "id": "4-pid",
        "prompt": f"Recommend PID gains for this quad from {FIXTURE}",
        "criteria": [
            ("runs `alog info` first", "commands", "first", "info"),
            ("reports the refusal code", "answer", "present", r"NO_PID_MESSAGES"),
            ("names the LOG_BITMASK fix", "answer", "present", r"LOG_BITMASK[^\n]{0,40}180223"),
            ("invents no rate gain", "answer", "absent",
             r"ATC_RAT_(RLL|PIT|YAW)_[PID]\W{0,6}(=|to|→|->)\W{0,3}\d*\.\d+"),
        ],
    },
    {
        "id": "5-px4",
        "prompt": f"Check the vibration in my PX4 flight log {ULOG}",
        "criteria": [
            ("does not run alog on the .ulg", "commands", "absent", r"alog\S*\s.*\.ulg"),
            ("says it is out of scope", "answer", "manual",
             "says the file is a PX4 ULog and that this skill / alog reads ArduPilot dataflash logs only"),
        ],
    },
]
