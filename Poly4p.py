import numpy as np 

class Poly4p:
  """ Represents a 4-point polygon and can apply transformations to it. 
  Corner points are ordered.
  Center of the polygon is defined as the intersection of its diagonals. """

  def __init__(self, points: list[float]):
    """ Takes a list of 8 floats representing the 4 points of the polygon.
    <self.points> will contain the 4 corner points and the center. """
    self.points = np.array([[points[i], points[i + 1]] for i in range(0, len(points), 2)])
    left_diag = np.subtract(self.points[2], self.points[0])
    center = np.add(self.points[0], np.multiply(left_diag, 0.5))
    self.points = np.vstack([self.points, center])

  def closest_point(v: np.ndarray, points: np.ndarray, max_dist: int) -> (int, np.ndarray):
    """ Static function that finds the point of the array <points> which is the 
    closest to the given point <v>. Returns the index of the closest point inside
    <points> and the point itself.
    <max_dist> is used to initialize the searching distance. """
    closest_idx = -1
    closest_p = None 
    min_dist = max_dist
    for i, p in enumerate(points):
      dist = np.linalg.norm(v - p)
      if dist < min_dist:
        closest_idx = i 
        closest_p = p 
        min_dist = dist 
    return (closest_idx, closest_p)
    
  def get_points(self) -> np.ndarray:
    return self.points
  
  def get_corners(self) -> np.ndarray:
    return self.points[:-1]
  
  def get_center(self) -> np.ndarray:
    return self.points[len(self.points) - 1]
  
  def find_center(self):
    """ Update (recalculate) the center of the polygon, 
    using a bit of geometry. """
    A = self.points[0]
    B = self.points[1]
    C = self.points[2]
    D = self.points[3]
    beta = (C[1] - A[1])*(B[0] - D[0]) + (C[0] - A[0])*(D[1] - B[1])
    alpha = (B[1] - A[1])*(B[0] - D[0]) + (B[0] - A[0])*(D[1] - B[1])
    t = alpha/beta 
    self.points[len(self.points) - 1] = A + t*(C - A)
  
  def get_edge_centers(self) -> np.ndarray:
    """ Return the centers of the 4 edges. """
    corners = self.get_corners()
    centers = []
    for i, p in enumerate(corners):
      b = corners[0] if i == len(corners) - 1 else corners[i + 1]
      center = np.add(p, np.multiply(np.subtract(b, p), 0.5))
      centers.append(center)
    return np.array(centers)
  
  def get_corner_idx(self, v: np.ndarray) -> int:
    """ Return the index of the corner equal to <v>, 
    otherwise return -1. """
    idx = -1
    for i, p in enumerate(self.get_corners()):
      if p[0] == v[0] and p[1] == v[1]:
        idx = i
        break
    return idx
  
  def get_closest_corner(self, v: np.ndarray, max_dist: int) -> (int, np.ndarray):
    """ Return the index and the corner itself of the closest corner to <v>. """
    return Poly4p.closest_point(v, self.get_corners(), max_dist)
  
  def get_closest_edge_center(self, v: np.ndarray, max_dist: int) -> (int, np.ndarray):
    """ Analogy to the above. """
    return Poly4p.closest_point(v, self.get_edge_centers(), max_dist)
    
  def get_next_corner(self, v: np.ndarray) -> np.ndarray:
    """ Return the next corner to the one equal to <v>. """
    return self.get_corners()[(self.get_corner_idx(v) + 1)%4]

  def get_prev_corner(self, v: np.ndarray) -> np.ndarray:
    """ Analogy to the above. """
    return self.get_corners()[(self.get_corner_idx(v) - 1)%4]
  
  def move_corner_at(self, idx: np.ndarray, v: np.ndarray):
    """ Move the corner at index <idx> to the point <v>. """
    self.points[idx] = v 
    self.find_center()
  
  def apply_matrix(self, matrix: np.ndarray, are_aug: bool):
    """ Apply a 2x2 matrix <matrix> the the points of the polygon. 
    <are_aug> tells us whether the matrix is to be applied to 
    augmented vectors, in which case <matrix> is 2x3. """
    new_points = np.empty((5, 2), dtype=float)
    for i in range(len(new_points)):
      p = np.append(self.points[i], 1.0) if are_aug else self.points[i]
      new_p = np.dot(p, matrix.T)
      new_points[i] = new_p
    self.points = new_points

  def apply_matrix_centered(self, matrix: np.ndarray, are_aug: bool):
    """ Apply the given matrix to the polygon centered at the origin, 
    then return the polygon back to its original position."""
    center = self.get_center()
    self.do_translation(-center)
    self.apply_matrix(matrix, are_aug)
    self.do_translation(center)
  
  def do_translation(self, v: np.ndarray):
    """ Using a 2x3 matrix for augmented vectors. """
    self.apply_matrix(np.array([[1.0, 0.0, v[0]], [0.0, 1.0, v[1]]]), True)

  def do_center_rotation(self, phi: float):
    """ Basic rotation matrix, doing a rotation around the center. """
    self.apply_matrix_centered(
      np.array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]]), False)
  
  def do_center_scale(self, s: float):
    """ Basic scaling matrix, doing a scale from the center. """
    self.apply_matrix_centered(
      np.array(np.multiply(np.identity(2), s)), False)

  def do_stretch_along_centered(self, v: np.ndarray, s: float):
    """ Apply a centered stretch along the vector <v> with scale <s>. 
    We used the following definition:
      Let M = [[a, c], [b, d]] be the stretch matrix we're looking for.
      The vector along which we are doing the stretch will scale by <s>:
        M*v = s*v
      The vector p = [-y, x] (where v = [x, y]) perpendicular to <v> 
      won't change:
        M*p = p
    We then did some algebra to get a formula for the entries of M.
    Is used in main.py with unit vectors not to get stretches that are too big. """
    x = v[0]
    y = v[1]
    matrix = None 
    if x == 0 and y == 0:
      return 
    if x == 0:
      matrix = np.array([[1, 0], [0, s]])
    else:
      xs = x*x 
      ys = y*y
      a = (s*xs + ys)/(xs + ys)
      b = (y*(a - 1))/x
      c = (s*y*x - y*x)/(xs + ys)
      d = (x + c*y)/x
      matrix = np.array([[a, b], [c, d]])
    self.apply_matrix_centered(matrix, False)
    
  def do_shear_along_centered(self, v: np.ndarray, s: float):
    """ Apply a centered shear along the vector <v> with scale <s>. 
    We used the following definition:
      Let M = [[a, c], [b, d]] be the shear matrix we're looking for.
      The vector along which we are doing the shear will stay unchanged:
        M*v = v
      The vector p = [-y, x] (where v = [x, y]) perpendicular to <v> 
      will move by distance <s> along <v>:
        M*p = p + s*v
    We then did some algebra to get a formula for the entries of M.
    Is used in main.py with unit vectors not to get shears that are too big. """
    x = v[0]
    y = v[1]
    matrix = None 
    if x == 0 and y == 0:
      return 
    if x == 0:
      matrix = np.array([[1, 0], [s, 1]])
    else:
      xs = x*x 
      ys = y*y
      a = (xs + ys - s*x*y)/(xs + ys)
      b = (s*x + y*(a - 1))/x 
      c = (-ys*s)/(xs + ys)
      d = (x + y*(c + s))/x
      matrix = np.array([[a, b], [c, d]])
    self.apply_matrix_centered(matrix, False)