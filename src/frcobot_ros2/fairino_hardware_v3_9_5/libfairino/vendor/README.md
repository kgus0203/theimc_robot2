# Bundled FAIRINO SDK 2.3.5 source

Copied from the local SDK source used to build the supplied libfairino.so.2.3.5:
/home/jeff/AGVSource/fairino/libfairino/src (2026-09-29).
Original source comments are retained. No files at that original location were modified.

Local fixes: CloseRPC stops and joins CNDE reception before FRRobot destruction;
CNDE/UDP receiver threads are owned and joined; invalid/truncated CNDE frames are rejected.
Builds as libfairino_checked_sdk.so to avoid accidentally loading the old binary.
