import tkinter as tk 
from Poly4p import * 
import misc 

SQUARE_RADIUS = 10

def render(c: tk.Canvas, render_poly, render_center, render_corners, render_stretchers):
  c.coords(render_poly, poly.get_corners())
  poly_center = poly.get_center()
  c.coords(render_center, misc.square_coords(poly_center[0], poly_center[1], SQUARE_RADIUS))
  corner_coords = poly.get_corners()
  for i in range(len(render_corners)):
    x = corner_coords[2*i]
    y = corner_coords[2*i + 1]
    c.coords(render_corners[i], misc.square_coords(x, y, SQUARE_RADIUS))
  stretcher_coords = poly.get_line_centers().flatten().tolist()
  for i in range(len(render_stretchers)):
    x = stretcher_coords[2*i]
    y = stretcher_coords[2*i + 1]
    c.coords(render_stretchers[i], misc.square_coords(x, y, SQUARE_RADIUS))

def init_render_poly(c: tk.Canvas, event, poly):
  if poly == None:
    return c.create_polygon(event.x, event.y, event.x, event.y, 
                            event.x, event.y, event.x, event.y, 
                            fill="black", outline="white", width=2)
    
def stretch_render_poly(c: tk.Canvas, event, poly, render_poly):
  if poly == None:
    x0, y0, _, _, _, _, _, _ = c.coords(render_poly)
    w = event.x - x0
    h = event.y - y0
    c.coords(render_poly, x0, y0, x0 + w, y0, event.x, event.y, x0, y0 + h) 

def drop_render_poly(event):
  global poly, render_center, render_poly, render_corners
  if poly == None:
    poly = Poly4p(c.coords(render_poly))
    poly_center = poly.get_center()
    render_center = c.create_rectangle(
      poly_center[0] - SQUARE_RADIUS, poly_center[1] - SQUARE_RADIUS,
      poly_center[0] + SQUARE_RADIUS, poly_center[1] + SQUARE_RADIUS,
      fill=None, outline="green", width=2, activefill="green", tags=("center",))
    corners = poly.get_corners()
    for i in range(0, len(corners) - 1, 2):
      x = corners[i]
      y = corners[i + 1]
      render_corners.append(c.create_rectangle(
        misc.square_coords(x, y, SQUARE_RADIUS),
        fill=None, outline="green", width=2, activefill="green", tags=("corner",)
      ))
    line_centers = poly.get_line_centers()
    for p in line_centers:
      render_stretchers.append(c.create_rectangle(
        misc.square_coords(p[0], p[1], SQUARE_RADIUS), 
        fill=None, outline="red", width=2, activefill="red", tags=("stretcher",)
      ))
    