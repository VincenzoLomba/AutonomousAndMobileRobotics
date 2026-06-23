
from pathlib import Path
from typing import Any, Dict
import json
import math
from dataclasses import fields
from geometry_msgs.msg import Pose
from . import tiagoExamParameters as params

def loadJSONFile(savedMapPath: str, filename: str) -> dict:
    # This simple method loads a JSON file from the given path and returns its content as a Python dictionary
    # If the file is not present or it not contains a JSON object (Python dictionary), an error is raised.
    # Otherwise, the content of the file is returned indeed as a Python dictionary.
    filePath = Path(savedMapPath) / filename
    if not filePath.exists(): raise FileNotFoundError(f"Required Task2 discovery file not found: {filePath}")
    if not filePath.is_file(): raise FileNotFoundError(f"Required Task2 discovery path is not a file: {filePath}")
    with open(filePath, "r", encoding="utf-8") as file: data = json.load(file)
    if not isinstance(data, dict): raise ValueError(f"Task2 discovery file '{filePath}' should contain a JSON object (dictionary), but it contains a {type(data).__name__}.")
    return data

def _requireField(data: Dict[str, Any], fileName: str, fieldName: str, label: str) -> Any:
    # This simple method can be used to check the presence of a required field in a dictionary ("fieald" is intended as "key" of the dictionary).
    # In case the required field is not present, an error is raised.
    # In case the required field is present, its value is returned.
    if fieldName not in data: raise ValueError(f"Missing required field '{fieldName}' in {label} location estimate (file '{fileName}').")
    return data[fieldName]

def _requireFiniteFloat(value: Any, fileName: str, fieldName: str, label: str) -> float:
    # This simple method can be used to check that a value is convertible to float and that it is finite.
    # In case of success, the value is returned as a float.
    # Otherwise, an error is raised.
    try: floatValue = float(value)
    except (TypeError, ValueError): raise ValueError(f"Field '{fieldName}' in {label} location estimate must be convertible to float (file '{fileName}').")
    if not math.isfinite(floatValue): raise ValueError(f"Field '{fieldName}' in {label} location estimate must be finite (file '{fileName}').")
    return floatValue

def _requireInt(value: Any, fileName: str, fieldName: str, label: str) -> int:
    # This simple method can be used to check that a value is convertible to int.
    # In case of success, the value is returned as an int.
    # Otherwise, an error is raised.
    try: return int(value)
    except (TypeError, ValueError): raise ValueError(f"Field '{fieldName}' in {label} location estimate must be convertible to int (file '{fileName}').")


def arucoPoseEstimateFromDict(data: Dict[str, Any], fileName: str, expectedLabel: str, expectedMarkerID: int) -> params.ArucoPoseEstimate:
    # This method can be used to load an ArUco location estimate from a dictionary (typically loaded from a JSON file).
    # IMPORTANT: this method is a "code monkey" one and I've written is exploting ChatGPT (giving to it che definition of the ArucoLocationEstimate class)!
    found = _requireField(data, fileName, "found", expectedLabel)
    if not isinstance(found, bool): raise ValueError(f"Field 'found' in {expectedLabel} location estimate must be a bool!")
    if not bool(found): raise ValueError(f"Task2 {expectedLabel} location estimate was not found (its 'found' field in file '{fileName}' is False).")
    
    requiredFields = {field.name for field in fields(params.ArucoPoseEstimate)} # This produces a set
    actualFields = set(data.keys())
    missingFields = requiredFields - actualFields
    if missingFields: raise ValueError(f"Missing required fields in {expectedLabel} location estimate: {sorted(missingFields)}.")

    for fieldName in requiredFields: _requireField(data, fileName, fieldName, expectedLabel)

    markerID = _requireInt(data["marker_id"], "marker_id", expectedLabel)
    label = str(data["label"])
    frameID = str(data["frame_id"])
    markerFrame = str(data["marker_frame"])
    sampleCount = _requireInt(data["sample_count"], "sample_count", expectedLabel)

    if label != expectedLabel: raise ValueError(f"Expected label '{expectedLabel}', got '{label}'.")
    if markerID != expectedMarkerID: raise ValueError(f"Expected marker_id {int(expectedMarkerID)}, got {markerID}.")
    if sampleCount <= 0: raise ValueError(f"{expectedLabel} location estimate must have sample_count > 0, got {sampleCount}.")

    rawPose = data["pose"]
    if not isinstance(rawPose, dict):
        raise ValueError(f"Field 'pose' in {expectedLabel} location estimate must be a dict.")

    expectedPoseFields = {"position", "orientation"}
    actualPoseFields = set(rawPose.keys())
    if actualPoseFields != expectedPoseFields:
        raise ValueError(
            f"Invalid pose fields in {expectedLabel} location estimate: "
            f"expected {sorted(expectedPoseFields)}, got {sorted(actualPoseFields)}."
        )

    rawPosition = rawPose["position"]
    rawOrientation = rawPose["orientation"]

    if not isinstance(rawPosition, dict):
        raise ValueError(f"Field 'pose.position' in {expectedLabel} location estimate must be a dict.")
    if not isinstance(rawOrientation, dict):
        raise ValueError(f"Field 'pose.orientation' in {expectedLabel} location estimate must be a dict.")

    expectedPositionFields = {"x", "y", "z"}
    actualPositionFields = set(rawPosition.keys())
    if actualPositionFields != expectedPositionFields:
        raise ValueError(
            f"Invalid pose.position fields in {expectedLabel} location estimate: "
            f"expected {sorted(expectedPositionFields)}, got {sorted(actualPositionFields)}."
        )

    expectedOrientationFields = {"x", "y", "z", "w"}
    actualOrientationFields = set(rawOrientation.keys())
    if actualOrientationFields != expectedOrientationFields:
        raise ValueError(
            f"Invalid pose.orientation fields in {expectedLabel} location estimate: "
            f"expected {sorted(expectedOrientationFields)}, got {sorted(actualOrientationFields)}."
        )

    pose = Pose()
    pose.position.x = _requireFiniteFloat(rawPosition["x"], "pose.position.x", expectedLabel)
    pose.position.y = _requireFiniteFloat(rawPosition["y"], "pose.position.y", expectedLabel)
    pose.position.z = _requireFiniteFloat(rawPosition["z"], "pose.position.z", expectedLabel)

    pose.orientation.x = _requireFiniteFloat(rawOrientation["x"], "pose.orientation.x", expectedLabel)
    pose.orientation.y = _requireFiniteFloat(rawOrientation["y"], "pose.orientation.y", expectedLabel)
    pose.orientation.z = _requireFiniteFloat(rawOrientation["z"], "pose.orientation.z", expectedLabel)
    pose.orientation.w = _requireFiniteFloat(rawOrientation["w"], "pose.orientation.w", expectedLabel)

    quaternionNorm = math.sqrt(
        pose.orientation.x * pose.orientation.x
        + pose.orientation.y * pose.orientation.y
        + pose.orientation.z * pose.orientation.z
        + pose.orientation.w * pose.orientation.w
    )

    if quaternionNorm <= 1e-9: raise ValueError(f"Orientation quaternion in {expectedLabel} location estimate has zero norm.")

    return params.ArucoPoseEstimate(
        found=found,
        marker_id=markerID,
        label=label,
        frame_id=frameID,
        marker_frame=markerFrame,
        sample_count=sampleCount,
        pose=pose,
    )

def loadTask2LocationEstimates(savedMapPath: str) -> tuple[params.ArucoPoseEstimate, params.ArucoPoseEstimate]:
    # This public method can be called in the Task3 FSM to retrive the estimated poses of the pick and place locations (in the map reference frame) 
    pickData = loadJSONFile(savedMapPath, params.pickLocationJSONFileName)
    placeData = loadJSONFile(savedMapPath, params.placeLocationJSONFileName)
    pickEstimate = arucoPoseEstimateFromDict(
        pickData,
        expectedLabel = params.pickLocationMarker.markerNickname,
        expectedMarkerID = params.pickLocationMarker.markerID,
    )
    placeEstimate = arucoPoseEstimateFromDict(
        placeData,
        expectedLabel = params.placeLocationMarker.markerNickname,
        expectedMarkerID = params.placeLocationMarker.markerID,
    )
    return pickEstimate, placeEstimate