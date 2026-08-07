#!/usr/bin/env python3
"""Check the C# sources against docs/API.md, and check the XML docs against the rules
the kRPC Python client actually applies.

Three things are verified, and each one has cost somebody a debugging session somewhere:

1.  Every kRPC member declared in C# appears in docs/API.md, and every member the docs
    claim exists really does. Documentation that quietly drifts from the code is worse
    than none, because a reader has no way to tell which half is wrong.

2.  No illegal type appears in a service signature. kRPC scans every loaded assembly at
    server start and ONE bad signature disables the entire server - every service, not
    just the offending one. build/scan catches this properly by running kRPC's own
    scanner, but that needs a KSP install; this is the cheap subset that runs in CI.

3.  Every file that states a version states the same one - CHANGELOG, the KSP-AVC
    .version, and each AssemblyInfo - and each plugin's KSPAssemblyDependency matches the
    Core's KSPAssembly. Drift means CKAN and KSP-AVC contradict each other in public, or
    KSP silently skips a plugin.

4.  The XML doc comments parse under the client's rules. Malformed XML or an unresolvable
    cref throws during kRPC's scan, which also takes the server down, and a bare
    <item> without a <description> raises IndexError inside the Python client.

Run:  python tools/check_docs.py
Exit: 0 if everything agrees.
"""

import io
import json
import os
import re
import sys
import glob
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# kRPC's own casing rules, from client/python/krpc/utils.py. Applied in this order.
def snake(name):
    s = re.sub(r"(.)_", r"\1__", name)
    s = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", s)
    s = re.sub(r"([A-Z]+)([A-Z][a-z0-9])", r"\1_\2", s)
    return s.lower()


# Types kRPC will accept in a signature. From KRPC.Service.TypeUtils.IsAValidType.
VALUE_TYPES = {
    "void", "bool", "int", "long", "uint", "ulong", "float", "double", "string", "byte[]",
}
COLLECTIONS = ("IList<", "IDictionary<", "HashSet<", "List<", "Dictionary<", "Tuple<")
MESSAGE_TYPES = {"KRPC.Service.Messages.Event"}

# Dictionary KEYS are further restricted - notably double and float are not allowed.
VALID_KEY_TYPES = {"int", "long", "uint", "ulong", "bool", "string"}

# Types that compile fine and then kill the kRPC server.
BANNED = {
    "Vector3", "Vector3d", "Quaternion", "Guid", "DateTime", "object", "byte", "short",
    "char", "decimal", "IEnumerable", "ISet", "ValueTuple",
}


def parse_sources():
    """Every kRPC member declared in src/, as {service: {py_name: (kind, type)}}."""
    services = {}
    member_re = re.compile(
        r"\[KRPC(Property|Procedure)[^\]]*\]\s*\n\s*"
        r"public static ([\w<>,\.\[\]\s]+?)\s+(\w+)\s*(?:\(([^)]*)\))?\s*(?:\{|$|\n)",
        re.MULTILINE,
    )

    for path in sorted(glob.glob(os.path.join(ROOT, "src", "**", "*.cs"), recursive=True)):
        src = io.open(path, encoding="utf-8").read()
        svc = re.search(r'\[KRPCService\s*\(\s*Name\s*=\s*"([^"]+)"', src)
        if not svc:
            continue
        name = svc.group(1)
        members = services.setdefault(name, {})
        for m in member_re.finditer(src):
            kind, rtype, member, args = m.group(1), m.group(2).strip(), m.group(3), m.group(4)
            members[snake(member)] = (kind, " ".join(rtype.split()), args or "", path)
    return services


def check_types(services):
    """Flag any signature type kRPC would reject."""
    problems = []
    for service, members in sorted(services.items()):
        for py_name, (kind, rtype, args, path) in sorted(members.items()):
            where = "%s.%s (%s)" % (service, py_name, os.path.relpath(path, ROOT))

            for text, what in ((rtype, "return type"), (args, "parameter")):
                if not text:
                    continue
                for banned in BANNED:
                    if re.search(r"\b%s\b" % re.escape(banned), text):
                        problems.append("%s: %s uses banned type '%s' -> %s"
                                        % (where, what, banned, text))

            base = rtype.split("<")[0].strip()
            legal = (rtype in VALUE_TYPES
                     or rtype in MESSAGE_TYPES
                     or rtype.startswith(COLLECTIONS)
                     or base in VALUE_TYPES)
            if not legal:
                problems.append("%s: return type '%s' is not one kRPC accepts" % (where, rtype))

            if rtype.startswith(("IDictionary<", "Dictionary<")):
                key = rtype.split("<", 1)[1].split(",", 1)[0].strip()
                if key not in VALID_KEY_TYPES:
                    problems.append("%s: dictionary key type '%s' is illegal (kRPC allows %s)"
                                    % (where, key, ", ".join(sorted(VALID_KEY_TYPES))))
    return problems


def check_xml_docs():
    """The two client-side parsing traps, checked statically."""
    problems = []
    for path in sorted(glob.glob(os.path.join(ROOT, "src", "**", "*.cs"), recursive=True)):
        rel = os.path.relpath(path, ROOT)
        lines = io.open(path, encoding="utf-8").read().splitlines()

        block, start = [], 0
        for number, line in enumerate(lines + [""], 1):
            stripped = line.strip()
            if stripped.startswith("///"):
                if not block:
                    start = number
                block.append(stripped[3:])
                continue
            if not block:
                continue

            doc = "\n".join(block)
            block = []

            # Well-formedness. kRPC parses this XML during its scan; a malformed
            # comment throws there and disables the server.
            try:
                node = ET.fromstring("<doc>%s</doc>" % doc)
            except ET.ParseError as exc:
                problems.append("%s:%d: XML doc comment does not parse: %s" % (rel, start, exc))
                continue

            # The Python client does item[0] on every <item>, so a bare <item>text</item>
            # raises IndexError on the client, far from here.
            for item in node.iter("item"):
                if len(list(item)) == 0:
                    problems.append(
                        "%s:%d: <item> without a child element - the kRPC Python client "
                        "does item[0] and will raise IndexError. Use "
                        "<item><description>...</description></item>." % (rel, start))

            # Tags the client silently drops at top level. Content written in one of
            # these never reaches help().
            for tag in ("example", "exception", "value", "seealso", "typeparam"):
                if node.find(tag) is not None:
                    problems.append(
                        "%s:%d: <%s> at top level is silently dropped by the kRPC Python "
                        "client - fold it into <summary>." % (rel, start, tag))
    return problems


def parse_api_doc():
    """Member names mentioned in docs/API.md, per service heading."""
    path = os.path.join(ROOT, "docs", "API.md")
    text = io.open(path, encoding="utf-8").read()

    # Service sections start at "## `conn.<name>`".
    sections, current = {}, None
    for line in text.splitlines():
        heading = re.match(r"^## `conn\.(\w+)`", line)
        if heading:
            current = heading.group(1)
            sections[current] = []
            continue
        if current:
            sections[current].append(line)

    mentioned, listed = {}, {}
    for service, body in sections.items():
        seen, rows = set(), set()
        in_member_table = False
        for line in body:
            # Anywhere in the section: `member`, `member(args)`.
            for token in re.findall(r"`([a-z_][a-z0-9_]*)\s*(?:\([^`]*\))?`", line):
                seen.add(token)

            # The reverse direction is checked only against tables that CATALOGUE
            # members, identified by a "Member" header. Prose is full of backticked
            # words that are not member names, and API.md also contains data tables -
            # the recovery-ledger categories, for instance - whose first column is
            # deliberately not a member name.
            if not line.strip().startswith("|"):
                in_member_table = False
                continue
            if re.match(r"\|\s*Member\s*\|", line):
                in_member_table = True
                continue
            if not in_member_table:
                continue
            cell = re.match(r"\|\s*`([a-z_][a-z0-9_]*)\s*(?:\([^`]*\))?`", line)
            if cell:
                rows.add(cell.group(1))

        mentioned[service] = seen
        listed[service] = rows
    return mentioned, listed


def check_versions():
    """Every place that states a version must state the same one.

    Four files carry it independently, and nothing but this ties them together. When they
    drift, CKAN reads one and KSP-AVC reads another, and the two contradict each other on
    a public page - the kind of inconsistency that makes people distrust the rest.

    Also checks that each plugin's KSPAssemblyDependency matches the Core's own
    KSPAssembly major and minor. Get that wrong and KSP does not error: it silently skips
    the plugin, and the only trace is one line in a 200 MB log.
    """
    problems = []
    seen = {}

    changelog = os.path.join(ROOT, "CHANGELOG.md")
    version = None
    if os.path.exists(changelog):
        for line in io.open(changelog, encoding="utf-8"):
            match = re.match(r"^##\s*\[([0-9]+\.[0-9]+\.[0-9]+)\]\s*[-—]?\s*(\S+)?", line)
            if match:
                version = match.group(1)
                seen["CHANGELOG.md"] = version
                date = match.group(2) or ""
                if not re.match(r"^\d{4}-\d{2}-\d{2}$", date):
                    problems.append(
                        "CHANGELOG.md: the top entry has no yyyy-mm-dd date - set it to "
                        "the day you actually publish")
                break
    if version is None:
        problems.append("CHANGELOG.md: no '## [x.y.z]' heading found")
        return problems

    avc = os.path.join(ROOT, "distribution", "KRPC.Bridge.version")
    if os.path.exists(avc):
        data = json.load(io.open(avc, encoding="utf-8"))
        v = data.get("VERSION", {})
        seen["KRPC.Bridge.version"] = "%s.%s.%s" % (v.get("MAJOR"), v.get("MINOR"), v.get("PATCH"))

    for path in sorted(glob.glob(os.path.join(ROOT, "src", "**", "AssemblyInfo.cs"),
                                 recursive=True)):
        rel = os.path.relpath(path, ROOT)
        text = io.open(path, encoding="utf-8").read()
        for attribute in ("AssemblyVersion", "AssemblyFileVersion"):
            match = re.search(r'%s\s*\(\s*"([0-9]+\.[0-9]+\.[0-9]+)' % attribute, text)
            if match:
                seen["%s (%s)" % (rel, attribute)] = match.group(1)

    wrong = {where: found for where, found in seen.items() if found != version}
    if wrong:
        problems.append("version mismatch - CHANGELOG.md says %s but:" % version)
        for where, found in sorted(wrong.items()):
            problems.append("    %-56s says %s" % (where, found))

    # KSPAssembly / KSPAssemblyDependency, which are a separate numbering entirely.
    core = os.path.join(ROOT, "src", "Core", "AssemblyInfo.cs")
    declared = None
    if os.path.exists(core):
        match = re.search(r'KSPAssembly\s*\(\s*"KRPC\.Bridge\.Core"\s*,\s*(\d+)\s*,\s*(\d+)',
                          io.open(core, encoding="utf-8").read())
        if match:
            declared = (match.group(1), match.group(2))
    if declared:
        for path in sorted(glob.glob(os.path.join(ROOT, "src", "Plugins", "*", "AssemblyInfo.cs"))):
            rel = os.path.relpath(path, ROOT)
            for line in io.open(path, encoding="utf-8"):
                if line.lstrip().startswith("//"):
                    continue
                match = re.search(
                    r'KSPAssemblyDependency\s*\(\s*"KRPC\.Bridge\.Core"\s*,\s*(\d+)\s*,\s*(\d+)',
                    line)
                if match and (match.group(1), match.group(2)) != declared:
                    problems.append(
                        "%s: depends on Core %s.%s but Core declares %s.%s - KSP will "
                        "silently SKIP this plugin"
                        % (rel, match.group(1), match.group(2), declared[0], declared[1]))
    return problems


def main():
    services = parse_sources()
    if not services:
        print("no kRPC services found in src/ - has the layout changed?")
        return 1

    mentioned, listed = parse_api_doc()
    problems = []

    total = 0
    for service, members in sorted(services.items()):
        total += len(members)
        py_service = snake(service)

        # Template is example code, not a shipped service.
        if service == "Template":
            continue

        if py_service not in mentioned:
            problems.append("docs/API.md has no '## `conn.%s`' section" % py_service)
            continue

        for name in sorted(set(members) - mentioned[py_service]):
            problems.append("docs/API.md: conn.%s.%s is not documented" % (py_service, name))

        # The reverse - something the docs promise that no longer exists - is checked
        # only against table rows, where members are catalogued. Prose is full of
        # backticked words that are not member names.
        for name in sorted(listed[py_service] - set(members)):
            problems.append("docs/API.md: conn.%s.%s is listed in a table but not declared "
                            "in C#" % (py_service, name))

    problems += check_types(services)
    problems += check_xml_docs()
    problems += check_versions()

    print("services: %s" % ", ".join("conn." + snake(s) for s in sorted(services)))
    print("kRPC members declared: %d" % total)
    print()

    if problems:
        print("%d problem(s):" % len(problems))
        for problem in problems:
            print("  " + problem)
        return 1

    print("OK - docs match the code, no illegal signature types, XML docs parse.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
