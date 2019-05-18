import IPython
from nose.tools import assert_equal, ok_, assert_almost_equal
from shapely.geometry import Point, Polygon, LineString, box


def check_path(path, bounds, environment, start, radius, goal_region):
    """Checks that the path is valid (except for collisions)."""

    minx, miny, maxx, maxy = bounds

    # Check path is a list
    ok_(isinstance(path, list), msg="path should be a list.")
    ok_(len(path)>0, msg="The returned path shouldn't be empty")

    # Check each element in the path is a tuple and is within bounds
    for i, path_pose in enumerate(path):
        ok_(isinstance(path_pose, tuple), msg="Each element of the path should be a tuple, element %d: %s is not." % (i, str(path_pose)))
        ok_(len(path_pose) == 2, msg="Each tuple of the path should have two elements, element %i: %s doesn't" % (i, str(path_pose)))
        x, y = path_pose
        ok_(minx <= x <= maxx and miny <= y <= maxy, msg="Element %i: %s is not within bounds" %(i, str(path_pose)))

    # Check start of path is start
    startx, starty = path[0]
    assert_almost_equal(startx, start[0], msg="The start of the path doesn't match the provided start (x is different)")
    assert_almost_equal(starty, start[1], msg="The start of the path doesn't match the provided start (y is different)")
    # Check end of path is in goal region
    endx, endy = path[-1]
    ok_(goal_region.contains(Point((endx,endy))), msg="The end of the path should be in the goal region.")

    try:
        from IPython.display import display_html
        display_html("""<div class="alert alert-success">
        <strong>Path seems to be correct.</strong>
        However, collisions are not checked. Make sure your path doesn't collide with any obstacles.
        </div>""", raw=True)
    except:
        print("test ok!!")