"""External OpenRocket geometry in metres, measured axially from the nose tip.

Supports the outer components present in the supplied single-stage design.
Internal mass placeholders are not invented as external mechanisms.
"""


def number(node, key, default=0):
    return float(node.findtext(key, str(default)).split()[-1])


def axial_start(node, parent_start, parent_length, length, previous_end):
    offset = node.find("axialoffset")
    if offset is None:
        return previous_end
    value = float(offset.text)
    method = offset.get("method")
    if method == "absolute":
        return value
    if method == "top":
        return parent_start + value
    if method == "middle":
        return parent_start + (parent_length - length) / 2 + value
    if method == "bottom":
        return parent_start + parent_length - length + value
    if method == "after":
        return previous_end + value
    raise ValueError("Unsupported ORK axial positioning: " + str(method))


def external_geometry(root):
    stage = root.find("./rocket/subcomponents/stage/subcomponents")
    parts = []
    end = 0.0
    for n in stage:
        if n.tag not in ("nosecone", "bodytube", "transition"):
            continue
        length = number(n, "length")
        start = axial_start(n, 0, 0, length, end)
        part = dict(kind=n.tag, name=n.findtext("name"), start_m=start, length_m=length)
        if n.tag == "nosecone":
            part.update(shape=n.findtext("shape"), aft_radius_m=number(n, "aftradius"))
        elif n.tag == "bodytube":
            part.update(radius_m=number(n, "radius"))
        else:
            part.update(
                shape=n.findtext("shape"),
                fore_radius_m=number(n, "foreradius"),
                aft_radius_m=number(n, "aftradius"),
            )
        parts.append(part)
        for child in n.findall("./subcomponents/*"):
            if child.tag == "trapezoidfinset":
                root_chord = number(child, "rootchord")
                parts.append(
                    dict(
                        kind="trapezoidfinset",
                        name=child.findtext("name"),
                        start_m=axial_start(child, start, length, root_chord, start),
                        root_chord_m=root_chord,
                        tip_chord_m=number(child, "tipchord"),
                        sweep_m=number(child, "sweeplength"),
                        span_m=number(child, "height"),
                        thickness_m=number(child, "thickness"),
                        count=int(number(child, "fincount")),
                        rotation_deg=number(child, "rotation"),
                        cant_deg=number(child, "cant"),
                        body_radius_m=part.get("radius_m", part.get("aft_radius_m", 0)),
                    )
                )
            elif child.tag == "launchlug":
                lug_length = number(child, "length")
                parts.append(
                    dict(
                        kind="launchlug",
                        name=child.findtext("name"),
                        start_m=axial_start(child, start, length, lug_length, start),
                        length_m=lug_length,
                        radius_m=number(child, "radius"),
                        angle_deg=number(child, "radialdirection"),
                        body_radius_m=part.get("radius_m", 0),
                    )
                )
        end = max(end, start + length)
    return dict(
        source="rocket.ork",
        units="m",
        length_m=end,
        parts=parts,
        note="External dimensions and placement from ORK; paint, fin airfoil profile, suspension/servo mounting and deployment separation are not specified by this visualization.",
    )
