
SHEAR = 0.1
def on_right(event):
  global poly 
  corner = poly.get_corner_points()[0]
  next_corner = poly.get_next_corner(corner)
  next_edge = next_corner - corner
  norm_next_edge = next_edge/np.linalg.norm(next_edge)
  angle_to_x_axis = np.arccos(np.dot(norm_next_edge, [1, 0]))
  poly.do_center_rotation(-angle_to_x_axis)
  poly.do_x_shear(-SHEAR)      
  poly.do_center_rotation(angle_to_x_axis)
  render()

def on_left(event):
  global poly 
  corner = poly.get_corner_points()[0]
  next_corner = poly.get_next_corner(corner)
  next_edge = next_corner - corner
  norm_next_edge = next_edge/np.linalg.norm(next_edge)
  angle_to_x_axis = np.arccos(np.dot(norm_next_edge, [1, 0]))
  poly.do_center_rotation(-angle_to_x_axis)
  poly.do_x_shear(SHEAR)      
  poly.do_center_rotation(angle_to_x_axis)
  render()

def get_corners(self) -> list[float]:
    ascending_x = sorted(self.points[:-1], key=lambda p: p[0])
    A = B = C = D = None
    if ascending_x[0][1] < ascending_x[1][1]:
      A = ascending_x[0]
      D = ascending_x[1]
    else:
      A = ascending_x[1]
      D = ascending_x[0]
    if ascending_x[2][1] < ascending_x[3][1]:
      B = ascending_x[2]
      C = ascending_x[3]
    else:
      B = ascending_x[3]
      C = ascending_x[2]
    return np.array([A, B, C, D]).flatten().tolist()

    L = 30

    if render_right_horn == None:
      render_right_horn = c.create_line(
        corner[0], corner[1], corner[0] + right_horn[0]*L, corner[1] + right_horn[1]*L, 
        width=2, fill="blue")
    else:
      c.coords(
        render_right_horn, corner[0], corner[1], corner[0] + right_horn[0]*L, corner[1] + right_horn[1]*L)

    if render_left_horn == None:
      render_left_horn = c.create_line(
        corner[0], corner[1], corner[0] + left_horn[0]*L, corner[1] + left_horn[1]*L, width=2, fill="yellow")
    else:
      c.coords(render_left_horn, corner[0], corner[1], corner[0] + left_horn[0]*L, corner[1] + left_horn[1]*L)
