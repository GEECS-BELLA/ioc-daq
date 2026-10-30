# ioc-daq

The multifunction-DAQ image of GEECS-BELLA's native EPICS IOCs:
epics-containers' `epics-base` developer image plus asyn, busy and calc from
upstream ibek-support recipes, and `epics-modules/measComp` (drvMultiFunction
only) over MCC's open uldaq library from our recipe in
`ibek-support-geecs/measComp/`. What the driver sends at start: `README.md`.

- **Instances** live in GEECS-BELLA/geecs-epics, not here: `daqtest1`
  (USB-1808X, `services/undulator/daq/daqtest1`, profile `bench-daq`).
  Records come from upstream's `<model>.substitutions`, loaded by the
  `measComp.MultiFunction` entity.
- **Why ours.** No epics-containers image or ibek-support recipe carries
  measComp (checked 2026-10-30). uldaq is MIT and built from source here, so
  the image may be published (no decision 12 restriction). A LabJack driver,
  if one is ever needed, joins this image (decision 11: named by family).
- **Images.** As ioc-plc: CI publishes on a git tag, CalVer `YYYY.M.N`
  (never re-push a tag), `ghcr.io/geecs-bella/ioc-daq-runtime:<tag>` and
  `-developer`, both made public once after the first tag. A pre-release tag
  (`2026.10.1b1`) for bench work from a branch. A local build:
  `DOCKER_DEFAULT_PLATFORM=linux/amd64 TARGET=runtime TAG=ioc-daq:local ./build`.
- **Pins.** geecs-epics' compose files pin the tag; a bump is a tag here and
  one line there, then `pull <service>` and recreate on the host.
- **Merging**: after a fresh-context review dispositioned on the PR, and the
  bench proof for anything that changes what reaches a device. Rules and
  program state: geecs-epics `CLAUDE.md`.
