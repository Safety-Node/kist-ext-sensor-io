// Deterministic probe of the DWM lec-line parser (no hardware needed).
//
// Checks: plain POS lines, POS embedded after DIST ranges, quality=0
// rejection, malformed input, and \r\n line reassembly across
// fragmented serial chunks.

#include "uwb/dwm_serial.hpp"

#include <cstdio>

using namespace kist;

static int g_failures = 0;

static void check(const char* name, bool ok) {
    std::printf("%-46s %s\n", name, ok ? "PASS" : "FAIL");
    if (!ok) ++g_failures;
}

int main() {
    // ── parse_pos_line ──────────────────────────────────────────
    {
        const auto s = parse_pos_line("POS,1.10,2.20,0.90,85");
        check("plain POS line parsed",
              s && s->x == 1.10f && s->y == 2.20f && s->z == 0.90f && s->quality == 85);
    }
    {
        const auto s = parse_pos_line("DIST,4,AN0,C584,1.00,2.00,0.50,2.34,AN1,8287,3.00,0.00,0.50,1.08,POS,0.62,1.42,0.90,52");
        check("POS after DIST ranges parsed",
              s && s->x == 0.62f && s->y == 1.42f && s->quality == 52);
    }
    {
        check("quality=0 rejected (no fix)", !parse_pos_line("POS,1.0,2.0,0.9,0"));
        check("negative coordinates accepted",
              parse_pos_line("POS,-3.5,-0.25,0.9,40").has_value());
        check("DIST-only line rejected", !parse_pos_line("DIST,4,AN0,C584,1.00,2.00,0.50,2.34"));
        check("truncated POS rejected", !parse_pos_line("POS,1.0,2.0"));
        check("garbage fields rejected", !parse_pos_line("POS,abc,2.0,0.9,50"));
        check("dwm> prompt rejected", !parse_pos_line("dwm> "));
    }

    // ── extract_lines: fragmented chunks ────────────────────────
    {
        std::string buf;
        buf += "POS,1.0,2.0,0.9,";           // first chunk: partial line
        auto lines = extract_lines(buf);
        check("partial line held back", lines.empty() && !buf.empty());

        buf += "85\r\nPOS,3.0";              // completes line 1, starts line 2
        lines = extract_lines(buf);
        check("completed line extracted",
              lines.size() == 1 && lines[0] == "POS,1.0,2.0,0.9,85");
        check("tail kept for next chunk", buf == "POS,3.0");

        buf += ",4.0,0.9,60\r\n\r\n";        // completes line 2 + empty line
        lines = extract_lines(buf);
        check("empty lines skipped, buffer drained",
              lines.size() == 1 && lines[0] == "POS,3.0,4.0,0.9,60" && buf.empty());
    }

    std::printf("\n%s (%d failures)\n", g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
    return g_failures == 0 ? 0 : 1;
}
