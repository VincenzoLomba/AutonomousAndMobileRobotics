
from pathlib import Path
from typing import Union, Optional
import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image
from scipy import ndimage
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra
from skimage.morphology import skeletonize
from .tiagoExamParameters import fatherReferenceFrame

def _neighbors8(y: int, x: int, shape: tuple[int, int]) -> list[tuple[int, int]]:
    # A simple method that retrieves the 8-connected neighbors of a given pixel (y, x) in a 2D array with a specified shape.
    out = []
    h, w = shape
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dy == 0 and dx == 0: continue
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w: out.append((ny, nx))
    return out

def _loadMap(yamlPath: Union[str, Path], yamlImageFieldName: str):
    # A very simple method that loads the map exploiting the related YAML file
    yamlPath = Path(yamlPath) # Ensure to work with a Path object
    with open(yamlPath, "r", encoding="utf-8") as f: yamlMetadata = yaml.safe_load(f) # load the content of the YAML file in a Python dictionary
    imagePath = yamlPath.parent / yamlMetadata[yamlImageFieldName] # The map YAML file (as produced by Ros2 Humble) is expected to have an "image" field
    imageArray = np.array(Image.open(imagePath)) # Retrive the image file content as a 2D numpy array (dtype uint8),
                                                 # where (accordingly to Ros2 Humble) each pixel value is expected to be either:
                                                 # 0 (occupied), 205 (unknown) or 254 (free)
    return yamlMetadata, imageArray

def _buildGreenSpace(imageArray: np.ndarray, mapResolution: float, robotRadius: float, occupiedCellValue: int, unknownCellValue: int, freeCellValue: int):
    # This method build the green space, AKA the set of cells where the robot can move without colliding with obstacles.
    # In doing that, the robot planar-base in considered as circular and with a specified radious.
    # Note that it's defined as "clearance", for each cell, the distance from the nearest non-traversable cell (occupied or unknown).
    # They are returned:
    # (1) greenMap: a boolean 2D array where True indicates cells that are in the green space
    # (2) clearanceMap: a 2D array of the same size of the map where each cell value is the clearance of that cell in METERS
    # (3) greenSpaceSemanticImage: a RGB image representing the computed green space in the original map
    occupied = imageArray == occupiedCellValue
    unknown = imageArray == unknownCellValue
    free = imageArray == freeCellValue
    nonTraversable = occupied | unknown
    clearanceMapPX = ndimage.distance_transform_edt(~nonTraversable) # This method computes the clearance, in pixels, for each single traversable cell
    clearanceMap = clearanceMapPX * mapResolution # Clearance in meters (remember: the clearance is the distance from the nearest non-traversable cell)
    greenMap = free & (clearanceMap >= robotRadius)
    greenSpaceSemanticImage = np.zeros((*imageArray.shape, 3), dtype=float) # Definition of an empty RGB image of the same size of the map
    greenSpaceSemanticImage[unknown] = [0.65, 0.65, 0.65] # gray
    greenSpaceSemanticImage[free] = [1.0, 1.0, 1.0] # white
    greenSpaceSemanticImage[occupied] = [0.0, 0.0, 0.0] # black
    greenSpaceSemanticImage[greenMap] = [0.88, 1.0, 0.88] # green
    return greenMap, clearanceMap, greenSpaceSemanticImage

def _computeJunctionsClearance(junctionsLabels: np.ndarray, junctionsAmount: int, clearanceMap: np.ndarray):
    # For each connected junction-area of the skeleton (AKA set of contiguous skeleton points with degree 3 or more), this method computes the related clearance.
    # That clearance is defined as the maximum clearance of the single skeleton points that are part of that junction-area.
    # The result is given in the form of a dictionary (keys: junctions labels, values: clearance in meters)
    junctionsClearance = {}
    for label in range(1, junctionsAmount + 1): # Iterate through all the junctions (note that the label 0 is reserved for pixels that are NOT part of any junction)
        pixels = junctionsLabels == label
        if np.any(pixels): junctionsClearance[label] = float(np.max(clearanceMap[pixels]))
    return junctionsClearance

def _skeletonizeGreenSpace(greenMap: np.ndarray, clearanceMap: np.ndarray):
    # The skeleton of the green space is computed. They are returned:
    # (1) skel [boolean 2D array]: the skeleton itself (True values indicate skeleton pixels)
    # (2) degree [2D array of integers]: for each skeleton pixel, the related degree (amount of OTHER skeleton pixels in an 8-connected 3x3 neighborhood)
    # (3) isolated [boolean 2D array]: skeleton pixels with degree 0
    # (4) endpoints [boolean 2D array]: skeleton pixels with degree 1
    # (5) junctionsLabels [2D array of integers]: labels for each connected component in the skeleton junctions, AKA for each junction (junctions are skeleton pixels with degree 3 or more)
    # (6) junctionsAmount [integer]: number of junctions
    # (7) junctionsClearance [dictionary]: clearance for each junction (keys: junctions labels, values: clearance in meters)
    # (8) chainsLabels [2D array of integers]: labels for each connected component in the skeleton chains, AKA for each chain (chains are skeleton pixels with degree 2)
    #                                    (note that pixels NOT correspinding to a skeleton chain are labelled as 0)
    # (9) chainsAmount [integer]: number of chains
    skel = skeletonize(greenMap)
    kernel = np.ones((3, 3), dtype=int)
    kernel[1, 1] = 0 # In computing the degree of a certain skeleton pixel, we DON'T want to consider the pixel itself 
    degree = ndimage.convolve(skel.astype(int), kernel, mode = "constant", cval = 0) # For each skeleton pixel, the related degree is computed
    # More specifically: the degree is the amount of OTHER skeleton pixels in an 8-connected 3x3 neighborhood.
    isolated = skel & (degree == 0)  # Isolated are skeleton pixels with degree 0
    endpoints = skel & (degree == 1) # Endpoints are skeleton pixels with degree 1
    chains = skel & (degree == 2)    # Chains are skeleton pixels with degree 2
    junctions = skel & (degree >= 3) # Junctions are skeleton pixels with degree 3 or more
    junctionsLabels, junctionsAmount = ndimage.label(junctions, structure = np.ones((3, 3), dtype = int)) # The skeleton junctions are labeled with a different integer for each connected component (AKA for each junction)
    junctionsClearance = _computeJunctionsClearance(junctionsLabels, junctionsAmount, clearanceMap)
    chainsLabels, chainsAmount = ndimage.label(chains, structure = np.ones((3, 3), dtype = int)) # The skeleton chains are labeled with a different integer for each connected component (AKA for each chain)
    return skel, degree, isolated, endpoints, junctionsLabels, junctionsAmount, junctionsClearance, chainsLabels, chainsAmount

def _backtrackEndpoints(startingPoint: tuple[int, int], skeleton: np.ndarray, degree: np.ndarray, clearanceMap: np.ndarray, clearanceThreshold: float):
    # A simple method that backtracks a specified point of the skeleton. The backtracking is performed in an iterative way untile one of the following conditions is met:
    # (1) the clearance threshold is met (i.e. the clearance of the current pixel is higher than the specified threshold)
    # (2) a junction is reached (more specifically, the degree of the current pixel is different from 2)
    # Note that is an isolated point is passed (AKA its degree is zero), then that point is immediatly returned
    # In the same way, if it is passed a point that is not and enpoint, then that point is immediatly returned
    current = startingPoint
    previous = None
    while True:
        cy, cx = current
        if clearanceMap[cy, cx] >= clearanceThreshold: return current # The current point matched the clearance threshold
        neighbors = [(ny, nx) for (ny, nx) in _neighbors8(cy, cx, skeleton.shape) if skeleton[ny, nx]] # Retrieve the skeleton neighbors of the current pixel, AKA ones related to the degree of the current point
        if previous is not None: neighbors = [p for p in neighbors if p != previous] # Exclude the previous pixel from the neighbors, in order to avoid going back and forth
        # In case we have no neighbors (AKA the given starting point is an isolated one), the method stops and returns the current point itself
        if len(neighbors) == 0: return current
        # In case we have more then one neighbor (having ALREADY excluded the "previous" neighbor, that is, more then two) (AKA we reached a junction), the method stops and returns the current point itself
        if len(neighbors) > 1: return current
        # Otherwise, we have exactly one neighbor (having ALREADY excluded the "previous" neighbor), so that "current" is a chain pixel (or the starting point IFF previous == None, that is, we still are in the first cycle of this for-loop): we keep backtracking by moving to that single not yet visited neighbor
        nxt = neighbors[0]
        ny, nx = nxt
        # One last check: if the degree of the next pixel is different from 2 (it is not a skeleton chain pixel), then we already know that we can stop and return the current pixel
        if degree[ny, nx] != 2: return current
        previous = current
        current = nxt

def _computeJunctionsKeypoints(junctionsLabels: np.ndarray, junctionsAmount: int, clearanceMap: np.ndarray):
    # For each junction area, this method extracts a keypoint as the one point of that junction area with the maximum clearance.
    # The returned keypoints are in pixel space as (row, col).

    junctionsKeypoints: list[tuple[int, int]] = []
    for label in range(1, junctionsAmount + 1): # Iterate through all the junctions (note that the label 0 is reserved for pixels that are NOT part of any junction)
        pixels = list(zip(*np.nonzero(junctionsLabels == label))) # Retrieve the pixels of the current junction area (as a list of (row, col) tuples)
        if not pixels: continue # A simple guard in case the current junction area is empty (this should NOT happen)
        keypoint = max(pixels, key=lambda p: float(clearanceMap[p[0], p[1]]))
        junctionsKeypoints.append(keypoint)
    return junctionsKeypoints

def _associatePointToJunctionArea(
        point: tuple[int, int],
        junctionLabels: np.ndarray,
        robotRadiusM: float,
        mapResolutionM: float,
        junctionsClearance: dict[int, float]
    ):
    # This method takes a single point as input and computes the junction area (if any) that is associated to that point.
    # The association is performed by checking the junctions that are within a radius equal to the robot radius (in pixels) from the given point.
    # The label of the associated junction area is returned. If the given point is not associated to any junction area, then 0 is returned.
    y, x = point
    radiusPx = int(np.ceil(robotRadiusM / mapResolutionM)) # Radius in pixels, given the robot radius in meters and the map resolution in meters/pixel
    labelsFound = set()
    for dy in range(-radiusPx, radiusPx + 1):
        for dx in range(-radiusPx, radiusPx + 1):
            if dx * dx + dy * dy > radiusPx * radiusPx: continue # This condition is needed to check points in a CIRCLE, and not in a SQUARE
            ny = y + dy
            nx = x + dx
            if ny < 0 or ny >= junctionLabels.shape[0]: continue # Check that the current point is within the map (.shape[0] is the amount of raws)
            if nx < 0 or nx >= junctionLabels.shape[1]: continue # Check that the current point is within the map (.shape[1] is the amount of columns)
            label = int(junctionLabels[ny, nx]) # Retrieve the junction label of the current point (0 will be retrieved IFF it is not part of any junction)
            if label > 0: labelsFound.add(label)
    if not labelsFound: return 0 # The given input point is NOT near any junction area
    return max(labelsFound, key=lambda label: junctionsClearance.get(label, 0.0))


def _mergeEndpointsByJunctionArea(
        refinedEnpointsCoordintaes: list[tuple[int, int]],
        clearanceMeters: np.ndarray, 
        junctionLabels: np.ndarray,
        robotRadius: float,
        mapResolutionM: float, 
        junctionsClearance: dict[int, float]
    ):
    # This method takes as input a list of refined endpoints coordinates and simply merges them by junction area.
    groupedAssociatedEndpoints: dict[int, list[tuple[int, int]]] = {} # A dictionary of lists. Keys: junction (area) labels. Values: enpoints associated to junctions.
    independent: list[tuple[int, int]] = [] # A list of enpoints NOT associated to any junction area.
    for pt in refinedEnpointsCoordintaes:
        associatedJuntionLabel = _associatePointToJunctionArea(pt, junctionLabels, robotRadius, mapResolutionM, junctionsClearance)
        if associatedJuntionLabel == 0: independent.append(pt)
        else: groupedAssociatedEndpoints.setdefault(associatedJuntionLabel, []).append(pt)

    mergedEndpointsCoordinates = independent.copy() # All inpedendent endpoints are kept as they are (since they are not associated to any junction area)
    for associatedJuntionLabel, pts in groupedAssociatedEndpoints.items(): # Iterating through all the junction areas (that have at least one associated endpoint)
        best = max(pts, key=lambda p: float(clearanceMeters[p[0], p[1]])) # Preserving, within the single junction area, only the associated endpoint with the highest clearance
        mergedEndpointsCoordinates.append(best)

    return mergedEndpointsCoordinates, groupedAssociatedEndpoints

def _pixelToMap(row: int, col: int, originX: float, originY: float, mapResolutionM: float, heightInCells: int):
    # This method convert a is explooted to convert a skeleton pixel in terms of (row, col) to a map-frame meters coordinates (x, y).
    # More specifically, this method has to be applied pixel-wise, with the pixel (row, col) that is converted to the map-frame coordinates (x, y) of the CENTER of that pixel.
    # That is: the final coordinates are given in METERS and are referenced with respect to the MAP FRAME!
    #
    # Indeed, accordingly to the ROS2 map_server convention (confirmed in nav2 map_io.cpp source):
    # (1) 'origin' in the YAML is the 2D pose of the lower-left corner of the image in the 'map' frame.
    # (2) map_server flips the PGM/image vertically before building the OccupancyGrid,
    # so that row 0 of the grid (the upper one) corresponds to the BOTTOM of the image (the lowest y in map frame).
    # (3) We load the PGM directly with PIL (as canonically done in Python, indeed expliting the library "from PIL import Image"),
    # where row 0 is the TOP of the image (the highest y in map frame).
    #
    # From these considerations, it follows the following:
    # (1) Retrieving the official Nav2 mapToWorld formula (costmap_2d.cpp / costmap_2d.py):
    #     wx = origin_x + (mx + 0.5) * resolution
    #     wy = origin_y + (my + 0.5) * resolution
    #     where (mx, my) are OccupancyGrid cell indices with my=0 at the BOTTOM (y increasing upward), and +0.5 places the coordinate at the cell center.
    #     indeed, (wx, wy) are the map-frame coordinates in meters of the center of the cell (mx, my).
    # (2) As said, we have a vertical flip between PIL image and OccupancyGrid:
    #     my = (height - 1) - rowPIL
    #     indeed, for an imahe with a certain height, PIL rows are indexed from 0 (top) to height-1 (bottom),
    #     while OccupancyGrid rows are indexed from 0 (bottom) to height-1 (top).
    #
    # Exploting the two above formulas, we derive the final formula to convert a pixel (row, col) to map-frame meters coordinates (x, y):
    mapX = originX + (col + 0.5) * mapResolutionM
    mapY = originY + (heightInCells - 0.5 - row) * mapResolutionM
    return float(mapX), float(mapY)

def _mapToPixel(mapX: float, mapY: float, originX: float, originY: float, mapResolutionM: float, heightInCells: int):
    # Simply the inverse of the "_mapToPixel" method above. It converts a map-frame meters coordinates (x, y) to a pixel (row, col).
    # Note that "round" is projecting to the nearest final cell.
    col = int(round((mapX - originX) / mapResolutionM - 0.5))
    row = int(round(heightInCells - 0.5 - (mapY - originY) / mapResolutionM))
    return row, col

def _skeletonShortestPathOrder(
        skel: np.ndarray,
        degree: np.ndarray,
        keypointCoordinatesPixel: list[tuple[int, int]],
        robotXYlocationPixel: tuple[int, int],
        mapResolutionM: float
    ):
    # This method implements a simple greedy (AKA optimizing only for the single step) ordering using shortest-path distance along the skeleton graph.
    # Firstly, the robot given position is projected to the nearest skeleton pixel.
    # Then, the nearest keypoint (in terms of shortest-path distance along the skeleton) is selected as the first one to visit.
    # The procedure is iterated until all keypoints are processed (excluding ones that cannot be reached, through the skeleton, starting from the robot position projection).
    # The final output will be a set of ordered keypoints indexes (the indexes are referred w.r.t the list "keypointCoordinatesPixel").
    #
    # Indeed, this ordering procedure involves repeatedly computing the shortest-path distance from the current skeleton pixel to all other skeleton keypoints,
    # treating the skeleton as a graph where each pixel is a node and edges/arcs exist between 8-connected skeleton pixels.
    # That is, the skeleton is explicitly converted to a graph representation, and a Dijkstra's algorithm is used to compute these shortest-path distances.
    #
    # Excluded keypoints (again, ones that cannot be reached, through the skeleton, starting from the robot position projection) will still be returned,
    # placed at the end of the ordered list, marked as "spurious", with "firstSpuriousIndex" indicating the index of the first spurious keypoint in the final list.
    # Note that all provided keypoints SHOULD lay on the skeleton. That said, kaypoints that are NOT laying on the skeleton are mmarked as "invalid" and also
    # added at the end of the ordered list, after the spurious ones.

    skel = skel & (degree > 0) # Excluding isolated skeleton pixels (degree 0)

    skelCoords = list(zip(*np.nonzero(skel))) # Retrieve the coordinates of all the skeleton pixels (in terms of (row, col) tuples)
    if not skelCoords or not keypointCoordinatesPixel: return [], 0 # A simple guard in case no skeleton pixels are present (this should NOT happen)
    # Convert the skeleton pixels to a dictionary; keys: skeleton pixel coordinates (row, col), values: unique integer ID for each skeleton pixel (analogous to an index)
    skeletonIDs = {p: i for i, p in enumerate(skelCoords)}

    # Project the robot position to the nearest skeleton pixel, and retrieve its index
    def nearestSkeletonPixel(pixel: tuple[int, int]) -> tuple[int, int]:
        py, px = pixel
        return min(skelCoords, key=lambda p: (p[0] - py) ** 2 + (p[1] - px) ** 2)
    currentID = skeletonIDs[nearestSkeletonPixel(robotXYlocationPixel)]

    validNodesIDs: list[int] = [] # This list will contain the IDs of the skeleton pixels that correspond to valid keypoints (so it will be related ONLY to keypoints).
                                  # Note that valid keypoints are those that actually are ON the skeleton (indeed, a keypoint not laying on the skeleton should not exist)
    validNodesIndexes: list[int] = [] # The indexes of the valid nodes (referred w.r.t. the list "keypointCoordinatesPixel")
    invalidNodesIndexes: list[int] = [] # The indexes of all the INvalid nodes (AKA keypoints that are not classified as valid)
    for i, kp in enumerate(keypointCoordinatesPixel):
        if kp in skeletonIDs:
            validNodesIDs.append(skeletonIDs[kp])
            validNodesIndexes.append(i)
        else: invalidNodesIndexes.append(i)

    # Defining all elements required by the Dijkstra's algorithm (scipy.sparse.csgraph.dijkstra)
    rows: list[int] = []
    cols: list[int] = []
    stepCosts: list[float] = []
    for row, col in skelCoords: # Iterating through all skeleton pixels
        skelPixelID = skeletonIDs[(row, col)] # Retriving the unique pixel ID of the current skeleton pixel
        for ny, nx in _neighbors8(row, col, skel.shape): # Retrieving the 8-connected neighbors of the current skeleton pixel and iterating through them
            if not skel[ny, nx]: continue # In case the neighbor is NOT a skeleton pixel, it is skipped
            destinationSkelPixelID = skeletonIDs[(ny, nx)] # Retriving the unique pixel ID of the current neighbor skeleton pixel
            step = np.sqrt(2.0) * mapResolutionM if (ny != row and nx != col) else mapResolutionM # Defining the step cost (AKA the arc cost): distance in meters
            # Finally, adding a new step/arc from the current skeleton pixel to the current neighbor skeleton pixel
            rows.append(skelPixelID)
            cols.append(destinationSkelPixelID)
            stepCosts.append(float(step))
    graph = csr_matrix((stepCosts, (rows, cols)), shape=(len(skelCoords), len(skelCoords))) # Defining the graph in a sparse matrix format

    # Actually ordering all the valid keypoints by iteratively computing the shortest-path distance (starting from the robot position projection)
    # List of NOT yet processed valid keypoints (initially: all of them) in terms of indexes to be used for the "validNodesIDs" list
    remainingIndexes = list(range(len(validNodesIDs)))
    # List (initially empty, to be progressively filled) of the ordered valid keypoints (again in terms of indexes to be used for the "validNodesIDs" list)
    ordered: list[int] = []
    while remainingIndexes:
        distances = dijkstra(graph, directed = False, indices = currentID) # Array of distances (indexed by the node/pixel ID)
        bestIndex = None
        bestDistance = float("inf")
        for rIndex in remainingIndexes:
            d = float(distances[validNodesIDs[rIndex]])
            if d < bestDistance:
                bestDistance = d
                bestIndex = rIndex
        if bestIndex is None or not np.isfinite(bestDistance): break # No more reachable valid keypoints (the remaining ones are indeed unreachable/spurious)
        remainingIndexes.remove(bestIndex) # Remove the "best" keypint to the "remainingIndexes" list, since it has been just processed
        ordered.append(bestIndex) # Add the "best" keypoint to the "orderd" list, since it is the next point to be orderly visited after the current one
        currentID = validNodesIDs[bestIndex] # Select that "best" point as the new current one (to be then processed)

    reachable = [validNodesIndexes[i] for i in ordered] # Indexes of the reachable valid keypoints (referred w.r.t. the list "keypointCoordinatesPixel")
    unreachable = [validNodesIndexes[i] for i in remainingIndexes] # Indexes of the unreachable/invalid keypoints (referred w.r.t. the list "keypointCoordinatesPixel")

    firstSpuriousIndex = len(reachable)
    orderedIndexes = reachable + unreachable + invalidNodesIndexes
    return orderedIndexes, firstSpuriousIndex

def _buildNavGoals(pointsXY: list[tuple[float, float]]):
    # This method simply converts a list of points (given as a set of (x, y) coordinates in meters, referred to the map frame)
    # to a list of Nav2 NavigateToPose goals (one per point, of type av2_msgs.action.NavigateToPose.Goal), that can be directly sent to a Nav2 Action Server.
    # As a convention, the orientation of each goal is set to an identity quaternion (x=0, y=0, z=0, w=1),
    # which in a planar envirnoment Nav2 interprets as yaw=0 (robot facing map +X / east).
    # Also, to be robust, this method checks whether the required ROS2 packages are available: if not, then an empty list is returned.
    try:
        from nav2_msgs.action import NavigateToPose
        from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
        from std_msgs.msg import Header
    except ImportError: return []
    nav2goals = []
    for x, y in pointsXY:
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped(
            header = Header(frame_id = fatherReferenceFrame),
            pose = Pose(
                position = Point(x = float(x), y = float(y), z = 0.0),
                orientation = Quaternion(x = 0.0, y = 0.0, z = 0.0, w = 1.0),
            ),
        )
        nav2goals.append(goal)
    return nav2goals

# This is the main method exposed by the Discovery Planner.
# The discovery plan is built accordingly to the following procedure:
# (1) The map of the environment is loaded, and the green space is computed.
#     The green space is the set of free cells where the robot can move without colliding with the present obstacles (e.g. walls or objects within the room).
#     In order to compute the green space, the robot is modelled as a circle with a specified radius.
# (2) The skeleton of the green space is computed, and its endpoints and junctions are extracted.
#     The skeleton is a one-pixel-wide representation of the green space, capturing its topology and connectivity.
#     The endpoints are the pixels of the skeleton that have only ONE neighbor.
#     The junctions are the pixels of the skeleton that have THREE OR MORE neighbors.
#     Conseguently, pixels with TWO neighbors are part of the skeleton chains between endpoints and junctions.
# (3) The endpoints are considered as keypoint candidates to be refined.
#     That refinement is done by backtracking along the skeleton towards the interior of the green space.
#     In performing this backtracking, is foundamental the concept of clearance:
#     the clearance of a pixel is the distance from that pixel to the nearest occupied/obstacle cell.
#     That said, the backtracking stops when a specified clearance threshold is met, which is defined as a percentage of the clearance range measured on skeleton pixels.
#     More specifically: given the minimumClearance and the maximumClearance measured ALL OVER the map, the threshold is defined as a percentage value
#     within the interval defined as [minimumClearance, maximumClearance].
# (4) Also junction areas are considered as kaypoints: for each junction area, the one cell with the maximum clearance is selected as a keypoint.
# (5) Once all endpoints have been backtracked/refined, they are also merged by junction area:
#     If multiple refined endpoints are associated to the same junction area, only the one with the highest clearance is kept as a keypoint, while the others are discarded.
#     An endpoint is associated to a junction area IFF it is within a radius equal to the robot radius from that junction area.
# (6) Finally, the remaining refined keypoints are ordered by shortest-path distance along the skeleton,
#     starting from the robot location projected to the nearest skeleton pixel, then returned.
#     In limit cases, during this reordering phase, some keypoints may appear to be NOT reachable through the skeleton from the robot location:
#     in that case, the returned "firstSpuriousIndex" indicates the index of the first spurious keypoint of the returned ordered list.
def computeOrderedKeypoints(
    yamlPath: Union[str, Path], # Path to the YAML map file (it can be BOTH a string or a Path object)
    robotXYlocation: tuple[float, float], # Robot location in the map in terms of (x, y) coordinates (meters)
    # In this Discovery Planner, the minimumClearance and the maximumClearance will be respectively defined as the minimum and maximum distance from the
    # map-skeleton pixels to the occupied/obstacle cells. That said, the clearancePercent parameter will be used to set the backtracking threshold as a
    # percentage within the interval defined as [minimumClearance, maximumClearance] (as already stated above).
    yamlImageFieldName: str = "image", # The field name within the YAML file that contains the name of the PGM image file (default used by ROs2 Humble is "image").
    yamlOriginFieldName: str = "origin", # The field name within the YAML file that contains the origin of the map (default used by Ros2 Humble is "origin").
    yamlResolutionFieldName: str = "resolution", # The field name within the YAML file that contains the resolution of the map (default used by ROs2 Humble is "resolution").
    clearancePercent: float = 50.0,
    robotRadius: float = 0.29,   # The robot is supposed to have a circular base/shape, with that specified radious (in meters).
    occupiedCellValue: int = 0,  # Pixel value for occupied cells in the map (default used by ROs2 Humble is 0).
    unknownCellValue: int = 205, # Pixel value for unknown cells in the map (default used by ROs2 Humble is 205).
    freeCellValue: int = 254,    # Pixel value for free cells in the map (default used by ROs2 Humble is 254).
    visualize: bool = False,     # In case this variable is set to True, then a set of plots showing the discovery process will be produced and properly saved.
) -> tuple[list[tuple[float, float]], list, int]:
    
    # The return value of this method is a tuple composed of three elements:
    # (1) orderedKeypoints [list of (float, float)]: keypoints in map-frame metric coordinates (x, y), ordered greedily by shortest-path distance along the skeleton.
    # (2) navGoals [list of NavigateToPose.Goal]: same list as before BUT expressed as Nav2 Action Goals (NavigateToPose.Goal objects)
    # (3) firstSpuriousIndex [int]: index of the first keypoint that was not reachable through the skeleton during the re-ordering phase.
    #                               Note that IFF all keypoints are valid/reachable, this value equals to len(ordered_keypoints).

    # Loading the map
    yamlMetadata, imageArray = _loadMap(yamlPath, yamlImageFieldName)
    mapResolution = float(yamlMetadata[yamlResolutionFieldName]) # The "resolution" is the amount of meters represented by each pixel in the map
    origin = yamlMetadata[yamlOriginFieldName]
    originX = float(origin[0]) # This value is the X coordinate of the origin of the map IN METERS
    originY = float(origin[1]) # This value is the Y coordinate of the origin of the map IN METERS
    imageHeight = imageArray.shape[0]

    # Computing the green space (free cells where the robot can fit without colliding with obstacles)
    greenMap, clearanceMapMeters, greenSpaceSemanticImage = _buildGreenSpace(imageArray, mapResolution, robotRadius, occupiedCellValue, unknownCellValue, freeCellValue)

    # Building up the skeleton and all related quantities
    skel, degree, isolated, endpoints, junctionsLabels, junctionsAmount, junctionsClearance, chainsLabels, chainsAmount = _skeletonizeGreenSpace(greenMap, clearanceMapMeters)

    # A simple guard: degenerate map with no skeleton at all
    if not skel.any(): return [], [], 0

    # Definition of the clearance threshold for backtracking/refinement of endpoints (isolated skeleton pixels - degree 0 - are exluded from this computation)
    validSkeleton = skel & (degree > 0)
    skeletonClearances = clearanceMapMeters[validSkeleton]
    minimumClearance = float(np.min(skeletonClearances))
    maximumClearance = float(np.max(skeletonClearances))
    thresholdClearanceMeters = minimumClearance + (clearancePercent / 100.0) * (maximumClearance - minimumClearance)

    # Endpoints backtracking/refinement (from endpoints):
    # for each endpoint, backtrack along the skeleton towards the interior of the green space until the clearance threshold is met.
    endpointCoordinates = list(zip(*np.nonzero(endpoints))) # Remember: "endpoints" is a binary/boolean 2D mask
    if not endpointCoordinates: return [], [], 0 # A simple guard: a degenerate map with no endpoints at all
    backtrackedEndpointsCoordinates = [_backtrackEndpoints(ep, skel, degree, clearanceMapMeters, thresholdClearanceMeters) for ep in endpointCoordinates]

    # Keypoints extraction from junctions:
    junctionsKeypointsCoordinates = _computeJunctionsKeypoints(junctionsLabels, junctionsAmount, clearanceMapMeters)
    unmergedKeypointsCoordinates = backtrackedEndpointsCoordinates + junctionsKeypointsCoordinates

    # Merging candidate keypoints by junction area: if multiple candidate keypoints are neighboring to the same junction area, only the one with the highest clearance is kept as a keypoint, while the others are discarded
    mergedKeypointsCoordinates, groupedAssociatedKeypoints = _mergeEndpointsByJunctionArea(
        unmergedKeypointsCoordinates, clearanceMapMeters, junctionsLabels, robotRadius, mapResolution, junctionsClearance
    )

    # Conversion of the finally computed endpoints from pixel coordinates to meters coordinates expressed in the MAP-frame
    keypointsCoordinatesInMAP: list[tuple[float, float]] = [
        _pixelToMap(row, col, originX, originY, mapResolution, imageHeight)
        for row, col in mergedKeypointsCoordinates
    ]

    # Computation of the location of the robot in terms of pixel coordinates
    robotXYlocationInPixels = _mapToPixel(robotXYlocation[0], robotXYlocation[1], originX, originY, mapResolution, imageHeight)

    # Ordering of the keypoints by shortest-path distance along the skeleton, starting from the robot location projected to the nearest skeleton pixel
    order, firstSpuriousIndex = _skeletonShortestPathOrder(skel, degree, mergedKeypointsCoordinates, robotXYlocationInPixels, mapResolution)
    orderedKeypoints = [keypointsCoordinatesInMAP[i] for i in order]

    # Converting the list of keypoints to a list of Nav2 NavigateToPose goals (one per keypoint)
    nav2goals = _buildNavGoals(orderedKeypoints)

    # Generates the discovery plots to visualize the discovery process
    if visualize:
        _saveDiscoveryPlots(
            yamlPath,
            imageArray,
            greenSpaceSemanticImage,
            endpoints,
            junctionsLabels,
            junctionsAmount,
            chainsLabels,
            chainsAmount,
            mergedKeypointsCoordinates,
            order,
            robotXYlocation,
            originX,
            originY,
            mapResolution,
            imageHeight,
            clearancePercent,
            thresholdClearanceMeters
        )

    return orderedKeypoints, nav2goals, firstSpuriousIndex

# The followwing code produces a set of meaningful plots to visualize the discovery process. It has benn ABSOLUTELY ChatGPT generated
# (I refuse to work explicitly with matplotlib and similars)
def _saveDiscoveryPlots(
        yamlPath: Union[str, Path],
        imageArray: np.ndarray,
        greenSpaceSemanticImage: np.ndarray,
        endpoints: np.ndarray,
        junctionsLabels: np.ndarray,
        junctionsAmount: int,
        chainsLabels: np.ndarray,
        chainsAmount: int,
        mergedKeypointsCoordinates: list[tuple[int, int]],
        order: list[int],
        robotXYlocation: tuple[float, float],
        originX: float,
        originY: float,
        mapResolutionM: float,
        imageHeight: int,
        clearancePercent: float,
        thresholdClearanceMeters: float,
    ) -> Path:

    outputDirectory = Path(yamlPath).parent / "discoveryPlots"
    outputDirectory.mkdir(parents=True, exist_ok=True)

    robotRow, robotCol = _mapToPixel(
        robotXYlocation[0],
        robotXYlocation[1],
        originX,
        originY,
        mapResolutionM,
        imageHeight,
    )

    # The YAML origin is the lower-left corner of the map in the map frame.
    # With imshow(origin="upper"), that corresponds to the lower-left image border.
    originCol = -0.5
    originRow = imageHeight - 0.5

    orderedMergedEndpointsCoordinates = [
        mergedKeypointsCoordinates[i]
        for i in order
        if 0 <= i < len(mergedKeypointsCoordinates)
    ]

    def _finishPlot(plotID: int) -> None:
        plt.tight_layout()
        plt.savefig(outputDirectory / f"{plotID}.png", dpi=150, bbox_inches="tight")
        plt.close()

    # ── 1. Original map ────────────────────────────────────────────────────────
    plt.figure(figsize=(8, 8))
    plt.imshow(imageArray, cmap="gray", origin="upper")
    plt.title("1: original map (raw PGM)")
    plt.axis("off")
    _finishPlot(1)

    # ── 2. Green space + robot spawn + map origin ──────────────────────────────
    plt.figure(figsize=(8, 8))
    plt.imshow(greenSpaceSemanticImage, origin="upper")

    plt.scatter(
        [robotCol],
        [robotRow],
        s=150,
        marker="x",
        c="red",
        linewidths=2.5,
        label=f"robot spawn ({robotXYlocation[0]:.2f}, {robotXYlocation[1]:.2f}) m",
        zorder=6,
    )

    plt.scatter(
        [originCol],
        [originRow],
        s=130,
        marker="o",
        facecolors="none",
        edgecolors="red",
        linewidths=2.0,
        label=f"map origin ({originX:.2f}, {originY:.2f}) m",
        zorder=7,
        clip_on=False,
    )

    plt.title("2: green space + robot spawn + map origin")
    plt.legend(loc="upper right", fontsize=8)
    plt.axis("off")
    _finishPlot(2)

    # ── 3. Skeleton chains + junction areas only ───────────────────────────────
    plt.figure(figsize=(8, 8))
    plt.imshow(greenSpaceSemanticImage, origin="upper")

    if chainsAmount > 0:
        ysChains, xsChains = np.nonzero(chainsLabels > 0)
        chainValues = chainsLabels[ysChains, xsChains]
        plt.scatter(
            xsChains,
            ysChains,
            c=chainValues,
            s=6,
            cmap="tab20",
            label="chains",
        )

    if junctionsAmount > 0:
        ysJunctions, xsJunctions = np.nonzero(junctionsLabels > 0)
        junctionValues = junctionsLabels[ysJunctions, xsJunctions]
        plt.scatter(
            xsJunctions,
            ysJunctions,
            c=junctionValues,
            s=30,
            marker="s",
            cmap="Set1",
            edgecolors="black",
            linewidths=0.5,
            label=f"junction areas ({junctionsAmount})",
            zorder=5,
        )

    plt.title("3: skeleton chains + junction areas")
    plt.legend(loc="upper right", fontsize=7)
    plt.axis("off")
    _finishPlot(3)

    # ── 4. Green space + skeleton chains / endpoints / junction areas ──────────
    plt.figure(figsize=(8, 8))
    plt.imshow(greenSpaceSemanticImage, origin="upper")

    if chainsAmount > 0:
        ysChains, xsChains = np.nonzero(chainsLabels > 0)
        chainValues = chainsLabels[ysChains, xsChains]
        plt.scatter(
            xsChains,
            ysChains,
            c=chainValues,
            s=6,
            cmap="tab20",
            label="chains",
        )

    if junctionsAmount > 0:
        ysJunctions, xsJunctions = np.nonzero(junctionsLabels > 0)
        junctionValues = junctionsLabels[ysJunctions, xsJunctions]
        plt.scatter(
            xsJunctions,
            ysJunctions,
            c=junctionValues,
            s=30,
            marker="s",
            cmap="Set1",
            edgecolors="black",
            linewidths=0.5,
            label=f"junction areas ({junctionsAmount})",
            zorder=5,
        )

    ysEndpoints, xsEndpoints = np.nonzero(endpoints)
    if len(xsEndpoints):
        plt.scatter(
            xsEndpoints,
            ysEndpoints,
            s=26,
            marker="o",
            facecolors="none",
            edgecolors="darkgreen",
            linewidths=1.8,
            label="original endpoints",
            zorder=6,
        )

    plt.title(
        f"4: original keypoints "
        f"(clearance threshold {clearancePercent:.0f}% → {thresholdClearanceMeters:.3f} m)"
    )
    plt.legend(loc="upper right", fontsize=7)
    plt.axis("off")
    _finishPlot(4)

    # ── 5. Refined keypoints + skeleton chains + junction areas ────────────────
    plt.figure(figsize=(8, 8))
    plt.imshow(greenSpaceSemanticImage, origin="upper")

    if chainsAmount > 0:
        ysChains, xsChains = np.nonzero(chainsLabels > 0)
        chainValues = chainsLabels[ysChains, xsChains]
        plt.scatter(
            xsChains,
            ysChains,
            c=chainValues,
            s=5,
            cmap="tab20",
            alpha=0.55,
            label="skeleton chains",
        )

    if junctionsAmount > 0:
        ysJunctions, xsJunctions = np.nonzero(junctionsLabels > 0)
        plt.scatter(
            xsJunctions,
            ysJunctions,
            c="black",
            s=5,
            alpha=0.55,
            label="junction areas",
            zorder=4,
        )

    if mergedKeypointsCoordinates:
        ysMerged = [p[0] for p in mergedKeypointsCoordinates]
        xsMerged = [p[1] for p in mergedKeypointsCoordinates]
        plt.scatter(
            xsMerged,
            ysMerged,
            s=60,
            marker="D",
            facecolors="red",
            edgecolors="white",
            linewidths=0.8,
            label=f"refined keypoints ({len(mergedKeypointsCoordinates)} total)",
            zorder=6,
        )

    plt.title(
        f"5: refined keypoints "
        f"(clearance threshold {clearancePercent:.0f}% → {thresholdClearanceMeters:.3f} m)"
    )
    plt.legend(loc="upper right", fontsize=8)
    plt.axis("off")
    _finishPlot(5)

    # ── 6. Ordered refined keypoints + robot spawn ─────────────────────────────
    plt.figure(figsize=(8, 8))
    plt.imshow(greenSpaceSemanticImage, origin="upper")

    if chainsAmount > 0:
        ysChains, xsChains = np.nonzero(chainsLabels > 0)
        chainValues = chainsLabels[ysChains, xsChains]
        plt.scatter(
            xsChains,
            ysChains,
            c=chainValues,
            s=5,
            cmap="tab20",
            alpha=0.55,
            label="_nolegend_",
        )

    if junctionsAmount > 0:
        ysJunctions, xsJunctions = np.nonzero(junctionsLabels > 0)
        plt.scatter(
            xsJunctions,
            ysJunctions,
            c="black",
            s=5,
            alpha=0.55,
            label="_nolegend_",
            zorder=4,
        )

    if orderedMergedEndpointsCoordinates:
        ysOrdered = [p[0] for p in orderedMergedEndpointsCoordinates]
        xsOrdered = [p[1] for p in orderedMergedEndpointsCoordinates]
        plt.scatter(
            xsOrdered,
            ysOrdered,
            s=60,
            marker="D",
            facecolors="red",
            edgecolors="white",
            linewidths=0.8,
            label="refined endpoints",
            zorder=6,
        )

        for visitIndex, (rowKeypoint, colKeypoint) in enumerate(
            orderedMergedEndpointsCoordinates,
            start=1,
        ):
            plt.text(
                colKeypoint + 4,
                rowKeypoint + 4,
                str(visitIndex),
                fontsize=7,
                color="red",
                fontweight="bold",
                ha="left",
                va="top",
                zorder=7,
            )

    plt.scatter(
        [robotCol],
        [robotRow],
        s=150,
        marker="x",
        c="blue",
        linewidths=2.5,
        label="robot spawn",
        zorder=8,
    )

    plt.title("6: ordered refined keypoints + robot spawn")
    plt.legend(loc="upper right", fontsize=8)
    plt.axis("off")
    _finishPlot(6)

    return outputDirectory