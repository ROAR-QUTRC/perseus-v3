"""URDF -> OpenRAVE native robot XML, for the BASE -> EEF chain only.

The openrave image has no collada_urdf, so we skip COLLADA entirely and write
OpenRAVE's own format. Only the kinematic chain matters to IKFast; geometry is
emitted as placeholder boxes so openrave-robot.py --info renders sensibly.
"""

import math
import sys
import xml.etree.ElementTree as ET


def rpy_to_axis_angle_deg(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    z = cr * cp * sy - sr * sp * cy
    yq = cr * sp * cy + sr * cp * sy
    n = math.sqrt(x * x + yq * yq + z * z)
    if n < 1e-12:
        return None
    angle = 2 * math.atan2(n, w)
    return (x / n, yq / n, z / n, math.degrees(angle))


def origin_of(el):
    o = el.find("origin")
    if o is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    xyz = tuple(float(v) for v in (o.get("xyz") or "0 0 0").split())
    rpy = tuple(float(v) for v in (o.get("rpy") or "0 0 0").split())
    return xyz, rpy


def frame_xml(xyz, rpy, indent):
    out = "%s<Translation>%.9g %.9g %.9g</Translation>\n" % ((indent,) + xyz)
    aa = rpy_to_axis_angle_deg(*rpy)
    if aa:
        out += "%s<rotationaxis>%.9g %.9g %.9g %.9g</rotationaxis>\n" % ((indent,) + aa)
    return out


def main(src, dst, base, eef, manip_dir):
    root = ET.parse(src).getroot()
    by_child = {j.find("child").get("link"): j for j in root.findall("joint")}

    chain, cur = [], eef
    while cur != base:
        if cur not in by_child:
            sys.exit("No path from %s up to %s (stuck at %s)" % (eef, base, cur))
        j = by_child[cur]
        chain.append(j)
        cur = j.find("parent").get("link")
    chain.reverse()

    out = ['<Robot name="%s">' % root.get("name"), "  <KinBody>"]
    out.append('    <Body name="%s" type="dynamic">' % base)
    out.append('      <Geom type="box"><Extents>0.02 0.02 0.01</Extents></Geom>')
    out.append("    </Body>")

    for j in chain:
        parent = j.find("parent").get("link")
        child = j.find("child").get("link")
        xyz, rpy = origin_of(j)
        out.append('    <Body name="%s" type="dynamic">' % child)
        out.append("      <offsetfrom>%s</offsetfrom>" % parent)
        out.append(frame_xml(xyz, rpy, "      ").rstrip("\n"))
        out.append('      <Geom type="box"><Extents>0.02 0.02 0.02</Extents></Geom>')
        out.append("    </Body>")

    for j in chain:
        name = j.get("name")
        jtype = j.get("type")
        parent = j.find("parent").get("link")
        child = j.find("child").get("link")
        axis = j.find("axis").get("xyz") if j.find("axis") is not None else "0 0 1"
        if jtype == "fixed":
            # OpenRAVE spells "rigidly attached" as a disabled hinge.
            out.append('    <Joint name="%s" type="hinge" enable="false">' % name)
            out.append("      <Body>%s</Body>" % parent)
            out.append("      <Body>%s</Body>" % child)
            out.append("      <offsetfrom>%s</offsetfrom>" % child)
            out.append("      <axis>%s</axis>" % axis)
            out.append("      <limits>0 0</limits>")
            out.append("    </Joint>")
            continue
        lim = j.find("limit")
        if jtype == "continuous":
            lo, hi = -math.pi, math.pi
        else:
            lo = float(lim.get("lower"))
            hi = float(lim.get("upper"))
        kind = "slider" if jtype == "prismatic" else "hinge"
        out.append('    <Joint name="%s" type="%s">' % (name, kind))
        out.append("      <Body>%s</Body>" % parent)
        out.append("      <Body>%s</Body>" % child)
        out.append("      <offsetfrom>%s</offsetfrom>" % child)
        out.append("      <axis>%s</axis>" % axis)
        if kind == "hinge":
            out.append(
                "      <limitsdeg>%.9g %.9g</limitsdeg>"
                % (math.degrees(lo), math.degrees(hi))
            )
        else:
            out.append("      <limits>%.9g %.9g</limits>" % (lo, hi))
        out.append("    </Joint>")

    out.append("  </KinBody>")
    out.append('  <Manipulator name="arm">')
    out.append("    <base>%s</base>" % base)
    out.append("    <effector>%s</effector>" % eef)
    out.append("    <direction>%s</direction>" % manip_dir)
    out.append("  </Manipulator>")
    out.append("</Robot>")

    open(dst, "w").write("\n".join(out) + "\n")
    movable = [j.get("name") for j in chain if j.get("type") != "fixed"]
    print(
        "    %d links, %d movable joints: %s"
        % (len(chain) + 1, len(movable), " -> ".join(movable))
    )
    for i, n in enumerate(movable):
        print("      freeindex %d = %s" % (i, n))


if __name__ == "__main__":
    main(*sys.argv[1:6])
