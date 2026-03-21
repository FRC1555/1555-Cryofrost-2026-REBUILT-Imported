# Log Review Drop Zone

Use this folder for robot logs you want Codex to inspect.

Recommended workflow:
- Copy only the specific log files you want reviewed into this folder.
- Keep large raw logs local by default so the repo history stays small.
- If you want to share a log in Git, add just that one file intentionally.

Useful file types:
- `*.wpilog`
- Driver Station logs
- AdvantageKit or AdvantageScope exports
- CSV telemetry exports

Good naming examples:
- `2026-03-21_practice_shooter-dip.wpilog`
- `2026-03-21_match3_brownout.csv`
- `2026-03-21_auto-start-test.dslog`

What to tell Codex:
- what the robot was doing
- what problem you saw
- which log file to inspect
- the rough timestamp or match number if you know it

Suggested prompts:
- `Analyze logs/2026-03-21_practice_shooter-dip.wpilog and tell me why the shooter slowed down`
- `Compare these two logs and tell me why auto was more stable in one run`
- `Check this log for brownouts, CAN issues, and shooter speed drop`
