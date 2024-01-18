import tkinter as tk 
import numpy as np 
import misc as misc 
from enum import Enum 
from Poly4p import *

""" ---------- INITIALIZE THE WINDOW AND THE CANVAS ---------- """

WIN_SIZE = 720 
root = tk.Tk()
root.title("2d rectangle transforms")
root.geometry(str(WIN_SIZE) + "x" + str(WIN_SIZE))
c = tk.Canvas(root, width=WIN_SIZE, height=WIN_SIZE, background="black")

""" ---------- MISC FUNCTIONS ---------- """

def square_coords(x, y, radius):
  """ Returns Canvas style-coordinates for a square. """
  return [x - radius, y - radius, x + radius, y + radius]

""" ---------- ENUMS ---------- """

class Transform(Enum):
  """ Enum defining the types of transformations. """
  TRANSLATION = 0
  RIGID = 1
  SIMILARITY = 2
  AFFINE = 3
  PERSPECTIVE = 4

class ShearType(Enum):
  ALONG_PREV = 0
  ALONG_NEXT = 1

""" ---------- CONSTANTS AND GLOBAL VARS ---------- """

SQUARE_RADIUS = 10
ROTATION_ANGLE = 0.05

transf = Transform.TRANSLATION
poly = None

render_poly = None
render_center = None
render_corners = []
render_stretchers = []

""" ---------- RENDERING FUNCTIONS ---------- """

def render():
  """ Render the polygon's visual by updating the coords 
  of the Canvas objects used to visualize it. """
  global render_poly, render_center, render_corners, render_stretchers

  c.coords(render_poly, poly.get_corners().flatten().tolist())

  poly_center = poly.get_center()
  c.coords(render_center, square_coords(poly_center[0], poly_center[1], SQUARE_RADIUS))

  for i, p in enumerate(poly.get_corners()):
    c.coords(render_corners[i], square_coords(p[0], p[1], SQUARE_RADIUS))
  
  for i, p in enumerate(poly.get_edge_centers()):
    c.coords(render_stretchers[i], square_coords(p[0], p[1], SQUARE_RADIUS))


def init_render_poly(event):
  """ Initialize the polygon's visual as a rectangle which will be stretched 
  by the user's mouse to draw the initial polygon. """
  global poly, render_poly
  if poly == None:
    render_poly = c.create_polygon(event.x, event.y, event.x, event.y, 
                                   event.x, event.y, event.x, event.y, 
                                   fill="black", outline="white", width=2)
    
def stretch_render_poly(event):
  """ Stretch the polygon's initial rectangle visual according to user's mouse. """
  global poly, render_poly
  if poly == None:
    x0, y0, _, _, _, _, _, _ = c.coords(render_poly)
    w = event.x - x0
    h = event.y - y0
    c.coords(render_poly, x0, y0, x0 + w, y0, event.x, event.y, x0, y0 + h) 

def drop_render_poly(event):
  """ User stopped drawing the initial rectangle visual. 
  Initialize the <Poly4p> object as well as the remaining visuals """
  global poly, render_center, render_poly, render_corners, render_stretchers
  if poly == None:
    poly = Poly4p(c.coords(render_poly))
    poly_center = poly.get_center()
    render_center = c.create_rectangle(
      square_coords(poly_center[0], poly_center[1], SQUARE_RADIUS),
      fill=None, outline="green", width=2, activefill="green", tags=("center",))
    
    for p in poly.get_corners():
      render_corners.append(c.create_rectangle(
        square_coords(p[0], p[1], SQUARE_RADIUS),
        fill=None, outline="green", width=2, activefill="green", tags=("corner",)
      ))

    for p in poly.get_edge_centers():
      render_stretchers.append(c.create_rectangle(
        square_coords(p[0], p[1], SQUARE_RADIUS), 
        fill=None, outline="red", width=2, activefill="red", tags=("stretcher",)
      ))
    
""" ---------- EVENT CALLBACKS (CONTROL THE TRANSFORMATIONS) ---------- """

def on_center_drag(event):
  """ User is dragging the center. Translate the polygon to the new mouse position. """
  global poly 
  poly.do_translation(np.array([event.x, event.y]) - poly.get_center())
  render()

def on_rotation_left(event):
  """ User pressed the left rotation button. """
  global poly, transf 
  if transf.value >= Transform.RIGID.value:
    poly.do_center_rotation(-ROTATION_ANGLE)
    render()

def on_rotation_right(event):
  """ User pressed the right rotation button. """
  global poly, transf 
  if transf.value >= Transform.RIGID.value:
    poly.do_center_rotation(ROTATION_ANGLE)
    render()

""" Global vars of the <on_corner_drag()> callback, used to keep track of the 
current shear happening. """
shear_happening = None 
shear_corner_idx = -1
last_v_event = None 

def on_corner_drag(event):
  """ User is dragging a corner of the polygon. Do the corresponding transformation. """
  global poly, render_corners, shear_happening, shear_corner_idx, last_v_event

  v_event = np.array([event.x, event.y])

  if transf.value >= Transform.PERSPECTIVE.value:
    poly.move_corner_at(poly.get_closest_corner(v_event, 2*WIN_SIZE)[0], v_event)

  elif transf.value >= Transform.AFFINE.value:
    # TODO Sometimes the shear and the mouse move in opposite directions. IDK why. 
    if shear_corner_idx == -1:  
      # no shear happening - find the closest corner to the mouse
      shear_corner_idx, corner = poly.get_closest_corner(v_event, 2*WIN_SIZE)
      last_v_event = v_event
    else:
      # shear already happening - update the concerned corner 
      corner = poly.get_corners()[shear_corner_idx]

    # calculate some useful vectors and values 
    next_edge = poly.get_next_corner(corner) - corner
    len_next_edge = np.linalg.norm(next_edge)
    norm_next_edge = next_edge/len_next_edge

    prev_edge = poly.get_prev_corner(corner) - corner 
    len_prev_edge = np.linalg.norm(prev_edge)
    norm_prev_edge = prev_edge/len_prev_edge

    if shear_happening == None:
      # mouse drag is the difference between the mouse position and the corner 
      drag = v_event - corner 
    else:
      # mouse drag is the difference between the last and current mouse position 
      drag = v_event - last_v_event
      last_v_event = v_event 
      
    # how close is the drag to the next edge 
    # TODO I think I could get rid of the minus sign here. 
    along_next_dot = np.dot(drag, -norm_next_edge)
    # how close is the drag to the prev edge 
    # TODO I think I could get rid of the minus sign here.  
    along_prev_dot = np.dot(drag, -norm_prev_edge)

    if shear_happening == None:
      # figure out along which edge the shear will be happening 
      if abs(along_next_dot) > abs(along_prev_dot):
        shear_happening = ShearType.ALONG_NEXT
      else:
        shear_happening = ShearType.ALONG_PREV

    along = None 
    scale = 0.0

    # set shear parameters 
    if shear_happening == ShearType.ALONG_NEXT:
      scale = 0 if along_next_dot == 0 else along_next_dot/len_next_edge
      along = norm_next_edge
    else:
      # We need a minus sign in the scale here because the Canvas coords 
      # are flipped compared to classic coords.
      scale = 0 if along_prev_dot == 0 else -along_prev_dot/len_prev_edge
      along = norm_prev_edge
    
    poly.do_shear_along_centered(along, scale)

  elif transf.value >= Transform.SIMILARITY.value:
    corner = poly.get_closest_corner(v_event, 2*WIN_SIZE)[1]
    corner_from_center = np.linalg.norm(poly.get_center() - corner)
    event_from_center = np.linalg.norm(poly.get_center() - v_event)
    poly.do_center_scale(event_from_center/corner_from_center)
  render()

def on_corner_drop(event):
  """ User dropped a corner of the polygon. Reset the global shear vars. """
  global shear_happening, shear_corner_idx
  shear_happening = None 
  shear_corner_idx = -1

def on_stretcher_drag(event):
  """ User is dragging a stretcher. Do a stretch. """
  if transf.value >= Transform.AFFINE.value:
    v_event = np.array([event.x, event.y])
    # find the edge center corresponding to the dragged stretcher 
    idx_edge_center, edge_center = poly.get_closest_edge_center(v_event, 2*WIN_SIZE)
    # figure out the scale of the stretch so that the stretcher stays on the mouse 
    stretcher_from_center = np.linalg.norm(poly.get_center() - edge_center)
    v_event_from_center = np.linalg.norm(poly.get_center() - v_event)
    scale = v_event_from_center/stretcher_from_center
    # find a vector perpendicular to the concerned edge 
    next_corner = poly.get_corners()[(idx_edge_center + 1)%4]
    u = next_corner - edge_center
    perp_to_edge = np.array([-u[1], u[0]])
    perp_to_edge = perp_to_edge/np.linalg.norm(perp_to_edge)

    poly.do_stretch_along_centered(perp_to_edge, scale)
  render()

def set_transf(event):
  """ User pressed a key in [0, 4]. Set the transformation accordingly. """
  global transf
  transf = Transform(int(event.char))
  print(transf)

""" ---------- BINDS ---------- """

transform_members = list(Transform.__members__.values())
for i in [mem.value for mem in transform_members]:
  root.bind(str(i), set_transf)

c.bind("<ButtonPress-1>", init_render_poly)
c.bind("<B1-Motion>", stretch_render_poly)  
c.bind("<ButtonRelease-1>", drop_render_poly)

c.tag_bind("center", "<B1-Motion>", on_center_drag)

c.tag_bind("corner", "<B1-Motion>", on_corner_drag)
c.tag_bind("corner", "<ButtonRelease-1>", on_corner_drop)

c.tag_bind("stretcher", "<B1-Motion>", on_stretcher_drag)

root.bind("q", on_rotation_left)
root.bind("e", on_rotation_right)

c.pack()
root.mainloop()