import os

from bokeh.embed import file_html
from bokeh.layouts import gridplot
from bokeh.models import Arrow, NormalHead
from bokeh.plotting import figure
from bokeh.resources import CDN
from html2image import Html2Image
from pathlib import Path

from compact_routing_structure import StraightBundle
from graph import Vertex
from obstacle import PointObstacle
from utils import vector_length

point_radius = 2  # The radius of the input vertices/obstacles
thin_edge_width = 8  # The width of the thin edges and displacement arrows
arrow_head_size = 15  # The size of the heads of the displacement arrows in pixels
end_time = 1  # The final event time of the growing algorithm


def visualize(instance, folder, filename, thick_edges=False, show_axes=False, show_delaunay=False,
              show_displacement=False, x_range=None, y_range=None):
    """
    Visualizes the given instance.

    :param instance: an Instance object
    :param folder: the folder in which the plot should be saved
    :param filename: the filename of the plot
    :param thick_edges: whether the instance should be drawn with thick edges
    :param show_axes: whether the axes of the plot should be drawn
    :param show_delaunay: whether the Delaunay triangulation on the vertices and obstacles should be drawn
    :param show_displacement: whether the displacement of the obstacles should be indicated using arrows
    :param x_range: the x-range of the plot
    :param y_range: the y-range of the plot
    """
    if x_range is None or y_range is None:
        plot = figure(match_aspect=True)
    else:
        plot = figure(match_aspect=True, x_range=x_range, y_range=y_range)

    if show_axes:
        plot.axis.visible = True
        plot.grid.visible = True
    else:
        plot.axis.visible = False
        plot.grid.visible = False

    # Draw edges
    for edge in instance.graph.edges:
        if thick_edges:
            straight_xs, straight_ys = [], []
            straight_outline_xs, straight_outline_ys = [], []

            elbow_xs, elbow_ys = [], []
            inner_radii, outer_radii = [], []
            start_angles, end_angles = [], []

            done = False
            prev_bundle = edge.elbow_bundle_v1
            current_bundle = prev_bundle.right
            while not done:
                if type(current_bundle) == StraightBundle:
                    # Set segment corners of straight
                    segment_corners = current_bundle.get_corners(end_time)
                    straight_xs.append([[[corner.x for corner in segment_corners]]])
                    straight_ys.append([[[corner.y for corner in segment_corners]]])

                    # Set outlines of straight
                    straight_outline_xs.append([segment_corners[0].x, segment_corners[1].x])
                    straight_outline_xs.append([segment_corners[2].x, segment_corners[3].x])
                    straight_outline_ys.append([segment_corners[0].y, segment_corners[1].y])
                    straight_outline_ys.append([segment_corners[2].y, segment_corners[3].y])

                    # If we arrive at the elbow bundle corresponding to v2, we are done
                    if current_bundle.right.is_terminal:
                        done = True
                else:
                    # Set center of annular wedge
                    elbow_xs.append(float(current_bundle.point.x))
                    elbow_ys.append(float(current_bundle.point.y))

                    # Set radii of annular wedge
                    inner_radii.append(end_time * current_bundle.layer_thickness)
                    outer_radii.append(end_time * (current_bundle.layer_thickness + current_bundle.thickness))

                    # Set angles of annular wedge
                    a1, a2 = current_bundle.get_angles(end_time)
                    start_angles.append(float(a2))
                    end_angles.append(float(a1))

                # Move to the next bundle of the thick edge
                current_b = current_bundle
                current_bundle = current_bundle.next(prev_bundle)
                prev_bundle = current_b

            # Draw straights
            plot.multi_polygons(straight_xs, straight_ys, line_color=edge.color, fill_color=edge.color)

            # Draw elbows
            plot.annular_wedge(elbow_xs, elbow_ys,
                               inner_radius=inner_radii, outer_radius=outer_radii,
                               start_angle=start_angles, end_angle=end_angles,
                               line_color=edge.color, fill_color=edge.color)

            # Draw outlines of straights
            plot.multi_line(straight_outline_xs, straight_outline_ys, line_color='black')

            # Draw outlines of elbows
            plot.arc(elbow_xs, elbow_ys,
                     radius=inner_radii,
                     start_angle=start_angles, end_angle=end_angles,
                     line_color='black')
            plot.arc(elbow_xs, elbow_ys,
                     radius=outer_radii,
                     start_angle=start_angles, end_angle=end_angles,
                     line_color='black')
        else:
            path = edge.path

            xs = [float(p.x) for p in path]
            ys = [float(p.y) for p in path]

            # Draw thin edge
            plot.line(xs, ys, line_width=thin_edge_width, color=edge.color)

    # Draw obstacles
    for obstacle in instance.obstacles:
        if type(obstacle) == PointObstacle:
            plot.circle(float(obstacle.x), float(obstacle.y), radius=point_radius, line_color='black',
                        fill_color=obstacle.fill_color)
        else:
            path = obstacle.path
            xs = [float(p.x) for p in path]
            ys = [float(p.y) for p in path]
            plot.multi_polygons([[[xs]]], [[[ys]]], line_color=obstacle.stroke_color, fill_color=obstacle.fill_color)

    # Draw vertices
    for vertex in instance.graph.vertices:
        radius = end_time * vertex.radius if thick_edges else point_radius
        plot.circle(float(vertex.x), float(vertex.y), radius=radius, color=vertex.color)

    if show_displacement:
        # Draw displacement arrows
        for point in instance.obstacles + instance.graph.vertices:
            if point.original_position != point:
                old_x = float(point.original_position.x)
                old_y = float(point.original_position.y)

                # Compute end of arrow such that it is attached to the boundary of the point
                vec = point - point.original_position
                dist = vector_length(vec)
                unit_vec = vec / dist
                if isinstance(point, Vertex):
                    arrow_end = point.original_position + unit_vec * (dist - point.radius)
                else:
                    arrow_end = point.original_position + unit_vec * (dist - point_radius)

                # Draw arrow
                arrow_head = NormalHead(size=arrow_head_size)
                plot.add_layout(Arrow(end=arrow_head, line_width=thin_edge_width, x_start=old_x, y_start=old_y,
                                      x_end=float(arrow_end.x), y_end=float(arrow_end.y)))

    if show_delaunay:
        # Draw Delaunay triangulation
        xs = []
        ys = []
        for triangle in instance.homotopy.dt.triangles:
            xs.append([[[float(he.origin.x) for he in triangle.half_edges]]])
            ys.append([[[float(he.origin.y) for he in triangle.half_edges]]])
        plot.multi_polygons(xs, ys, line_color='black', fill_alpha=0)

    # Save plot as png
    full_screen_plot = gridplot([[plot]], toolbar_location=None, sizing_mode='stretch_both')
    html = file_html(full_screen_plot, CDN)
    hti = Html2Image()
    hti.screenshot(html_str=html, save_as=f'{instance.name}.png')

    Path(folder).mkdir(parents=True, exist_ok=True)
    os.replace(f'{instance.name}.png', f'{folder}/{filename}.png')
