namespace KRPC.Bridge.Core
{
    /// <summary>
    /// The schema descriptors of the Core's own text tables (manifest_v1, clock_v1, vessels_v1,
    /// bits_v1, bit_sets_v1, layout_v1, layouts_v1), declared into the layout registry at boot.
    /// A text table is tab-separated: a "header" section of key=value pairs (row 0), a
    /// "columns" row naming the columns (row 1), then rows until the end (count "*").
    /// </summary>
    internal static class CoreLayouts
    {
        const string Service = "Bridge";
        const string Owner = "Core";

        internal static void Declare ()
        {
            var m = LayoutRegistry.Declare ("manifest_v1", Service, Owner, BridgeManifest.Protocol,
                BridgeManifest.Schema, LayoutRegistry.KindText);
            m.Once ("header", "protocol", "schema", "n", "assemblies", "git_sha", "git_dirty", "consistent");
            m.Rows ("columns", "*", BridgeManifest.Columns.Length.ToString (System.Globalization.CultureInfo.InvariantCulture),
                BridgeManifest.Columns);

            var c = LayoutRegistry.Declare ("clock_v1", Service, Owner, GameClock.Protocol, 1, LayoutRegistry.KindText);
            c.Once ("header", "protocol", "tick", "fixed_time", "ut", "dt", "warp_rate", "real_time", "hooked",
                "early_opens", "core_opens", "n");
            c.Rows ("columns", "*", "3", "name", "value", "last_tick");

            var v = LayoutRegistry.Declare ("vessels_v1", Service, Owner, VesselRegistry.Protocol, 1,
                LayoutRegistry.KindText);
            v.Once ("header", "protocol", "tick", "active", "n");
            v.Rows ("columns", "*", VesselRegistry.Columns.Length.ToString (System.Globalization.CultureInfo.InvariantCulture),
                VesselRegistry.Columns);

            var b = LayoutRegistry.Declare ("bits_v1", Service, Owner, BitRegistry.Protocol, 1, LayoutRegistry.KindText);
            b.Once ("header", "protocol", "set", "owner", "n");
            b.Rows ("columns", "*", BitRegistry.Columns.Length.ToString (System.Globalization.CultureInfo.InvariantCulture),
                BitRegistry.Columns);

            var s = LayoutRegistry.Declare ("bit_sets_v1", Service, Owner, BitRegistry.Protocol, 1,
                LayoutRegistry.KindText, "no header: one row per set");
            s.Rows ("rows", "*", "3", "name", "owner", "bits");

            var l = LayoutRegistry.Declare ("layout_v1", Service, Owner, LayoutRegistry.Protocol, 1,
                LayoutRegistry.KindText, "after the header, '#section' lines (name, count, stride, prefixed, " +
                "frame) each followed by its field lines (section, offset, name, unit)");
            l.Once ("header", "protocol", "layout", "service", "layout_protocol", "layout_schema", "kind",
                "sections", "fields", "note");
            l.Rows ("sections", "*", "6", "marker", "name", "count", "stride", "prefixed", "frame");
            l.Rows ("fields", "*", "4", "section", "offset", "name", "unit");

            var ls = LayoutRegistry.Declare ("layouts_v1", Service, Owner, LayoutRegistry.Protocol, 1,
                LayoutRegistry.KindText, "no header: one row per layout");
            ls.Rows ("rows", "*", "6", "name", "service", "protocol", "schema", "kind", "fields");

            // Refactor lot 3 (3.3): the event bus V2, poll_events_v2. ut and value are SI (s, and the
            // unit each kind documents: N for part.joint_break, the warp rate, a count...).
            var e = LayoutRegistry.Declare ("events_v2", Service, Owner, EventLogV2.Protocol, EventLogV2.Schema,
                LayoutRegistry.KindText, "poll_events_v2; one call per client tick from the last seq seen");
            e.Once ("header", EventLogV2.HeaderKeys);
            e.Rows ("columns", "*", EventLogV2.Columns.Length.ToString (System.Globalization.CultureInfo.InvariantCulture),
                EventLogV2.Columns);
            LayoutRegistry.Expect (e, "header", EventLogV2.HeaderKeys.Length);
            LayoutRegistry.Expect (e, "columns", EventLogV2.Columns.Length);
        }
    }
}
