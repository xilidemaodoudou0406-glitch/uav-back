# Navigation assurance API

All endpoints use the platform Bearer token. Validation permits simulation only. No endpoint promotes calibration to field-verified or enables hardware flight.

| Endpoint | Request / response |
| --- | --- |
| `GET /api/navigation/catalog` | Software profiles, readable model datasets, `realFlightEnabled:false`. |
| `GET /api/navigation/example` | Clearly simulated control/checkpoint input. |
| `GET /api/navigation/calibrations` | Current user's immutable calibration profiles. |
| `POST /api/navigation/calibrations` | `name`, `origin`, `dataSource`, `controlPoints`, `checkPoints`; each point contains `name`, `model`, `reference`, with WGS84 `lat/lng/altitude`. |
| `POST /api/navigation/height/convert` | `point`, `source`, `target`, optional `takeoffEllipsoidHeightMeters`, `geoidUndulationMeters`. Missing required datum returns 422. |
| `POST /api/missions/{id}/validate` | Persistent report bound to snapshot checksum, errors/warnings, derived metrics, effective margins and data fingerprints. |

Mission creation additionally accepts `calibrationId`, explicit source `heightReference`, datum values, `validationSettings`, `errorScenario`. Default source is `model_ellipsoid_display`. Non-model input requires an owner-scoped calibration and is converted back to model display coordinates; `coordinateContext` freezes provenance. Old missions remain readable.

Settings include `profileId` (`simulation`, `mini3_lab`), horizontal/vertical minimum margins, body/model/position/tracking errors, latency, braking inclusion, `requireCoverage`, selected `obstacleDatasets` (`out2`,`reconstruction1`), and typed `noFlyZones` (`minLat,maxLat,minLng,maxLng,minAlt,maxAlt`). Speed and acceleration use ECEF/ENU-derived differences rather than claimed sample speed.

Error scenario: `eastBiasMeters`, `northBiasMeters`, `upBiasMeters`, eastward `driftMetersPerSecond`, per-axis deterministic `noiseAmplitudeMeters`, actual queued `telemetryDelayMs`. Error upper bounds and delay enter validation margins. Delayed packets preserve capture time and include actual source age; expired positions do not refresh freshness.

PREPARE always recomputes validation before dispatch. Model data missing/corrupt, coverage gaps, collision or exhausted query budget fail closed. Conservative geometry enclosures cover trajectory simplification and geographic interpolation; voxel candidates are tested against expanded cells. Coverage refers to metadata extents, not guaranteed survey completeness or solid-building interior classification.

The `mission_validations` and `navigation_calibrations` tables are additive; existing task/run data are retained. Validation reports are append-only. Model data fingerprinting uses manifest content + file size/mtime, not full file hashing every prepare.
