// Unit tests for the pure config/pipeline helpers. These cover the bits that have
// been fragile in the field: mountpoint slashes, tolerant numeric parsing, and
// encoder/codec selection.
#include <gtest/gtest.h>
#include <image2rtsp/param_utils.h>

using namespace image2rtsp::params;
using XmlRpc::XmlRpcValue;

// ---- normalize_mountpoint ---------------------------------------------------
TEST(NormalizeMountpoint, AddsLeadingSlash) {
    EXPECT_EQ(normalize_mountpoint("backcam"), "/backcam");
}
TEST(NormalizeMountpoint, KeepsLeadingSlash) {
    EXPECT_EQ(normalize_mountpoint("/backcam"), "/backcam");
}
TEST(NormalizeMountpoint, ConvertsBackslash) {
    EXPECT_EQ(normalize_mountpoint("\\backcam"), "/backcam");
    EXPECT_EQ(normalize_mountpoint("cam\\1"), "/cam/1");
}
TEST(NormalizeMountpoint, TrimsWhitespace) {
    EXPECT_EQ(normalize_mountpoint("  /back  "), "/back");
    EXPECT_EQ(normalize_mountpoint("\tback\n"), "/back");
}
TEST(NormalizeMountpoint, EmptyBecomesRoot) {
    EXPECT_EQ(normalize_mountpoint(""), "/");
    EXPECT_EQ(normalize_mountpoint("   "), "/");
}
TEST(NormalizeMountpoint, KeepsNestedPath) {
    EXPECT_EQ(normalize_mountpoint("/a/b/c"), "/a/b/c");
}

// ---- member_int (tolerant) --------------------------------------------------
static XmlRpcValue makeStruct() {
    XmlRpcValue s;
    s.begin();  // force TypeStruct
    return s;
}

TEST(MemberInt, ReadsInt) {
    XmlRpcValue s = makeStruct();
    s["bitrate"] = 800;
    EXPECT_EQ(member_int(s, "bitrate", 500), 800);
}
TEST(MemberInt, DefaultsWhenAbsent) {
    XmlRpcValue s = makeStruct();
    s["other"] = 1;
    EXPECT_EQ(member_int(s, "bitrate", 500), 500);
}
TEST(MemberInt, AcceptsQuotedString) {
    XmlRpcValue s = makeStruct();
    s["bitrate"] = std::string("1500");
    EXPECT_EQ(member_int(s, "bitrate", 500), 1500);
}
TEST(MemberInt, AcceptsDouble) {
    XmlRpcValue s = makeStruct();
    s["bitrate"] = 2500.0;
    EXPECT_EQ(member_int(s, "bitrate", 500), 2500);
}
TEST(MemberInt, DefaultsOnGarbageString) {
    XmlRpcValue s = makeStruct();
    s["bitrate"] = std::string("fast");
    EXPECT_EQ(member_int(s, "bitrate", 500), 500);
}

// ---- to_string / member_string ---------------------------------------------
TEST(ToString, HandlesScalarTypes) {
    XmlRpcValue vs = std::string("hi"); EXPECT_EQ(to_string(vs), "hi");
    XmlRpcValue vi = 42;                 EXPECT_EQ(to_string(vi), "42");
    XmlRpcValue vb = true;               EXPECT_EQ(to_string(vb), "true");
}
TEST(MemberString, DefaultsWhenAbsent) {
    XmlRpcValue s = makeStruct();
    s["type"] = std::string("topic");
    EXPECT_EQ(member_string(s, "type", "cam"), "topic");
    EXPECT_EQ(member_string(s, "missing", "def"), "def");
}

// ---- payloader_tail ---------------------------------------------------------
TEST(PayloaderTail, H264ByDefault) {
    EXPECT_NE(payloader_tail("").find("rtph264pay name=pay0"), std::string::npos);
    EXPECT_NE(payloader_tail("h264").find("h264parse"), std::string::npos);
}
TEST(PayloaderTail, H265) {
    EXPECT_NE(payloader_tail("h265").find("rtph265pay name=pay0"), std::string::npos);
    EXPECT_NE(payloader_tail("hevc").find("h265parse"), std::string::npos);
}

// ---- encoder_fragment -------------------------------------------------------
TEST(EncoderFragment, X264Default) {
    std::string f = encoder_fragment("", "", "500");
    EXPECT_NE(f.find("x264enc"), std::string::npos);
    EXPECT_NE(f.find("bitrate=500"), std::string::npos);
    EXPECT_NE(f.find("video/x-h264"), std::string::npos);
}
TEST(EncoderFragment, NamesEncoderForLiveRetune) {
    // Every encoder must carry name=venc0 so the nodelet can grab it and change
    // bitrate live (dynamic_reconfigure). Cover software + hardware, h264 + h265.
    EXPECT_NE(encoder_fragment("x264", "h264", "500").find("name=venc0"), std::string::npos);
    EXPECT_NE(encoder_fragment("x264", "h265", "500").find("name=venc0"), std::string::npos);
    EXPECT_NE(encoder_fragment("nvenc", "h264", "500").find("name=venc0"), std::string::npos);
    EXPECT_NE(encoder_fragment("nvenc", "h265", "500").find("name=venc0"), std::string::npos);
}
TEST(EncoderFragment, UnknownFallsBackToX264) {
    std::string f = encoder_fragment("magic", "h264", "500");
    EXPECT_NE(f.find("x264enc"), std::string::npos);
}
TEST(EncoderFragment, Nvenc) {
    std::string f = encoder_fragment("nvenc", "h264", "800");
    EXPECT_NE(f.find("nvh264enc"), std::string::npos);
    EXPECT_NE(f.find("bitrate=800"), std::string::npos);
}
TEST(EncoderFragment, H265Software) {
    std::string f = encoder_fragment("x264", "h265", "500");
    EXPECT_NE(f.find("x265enc"), std::string::npos);
    EXPECT_NE(f.find("video/x-h265"), std::string::npos);
    // Must pin I420 so x265enc's default (main) profile can encode - videoconvert
    // would otherwise negotiate Y444 and media-prepare fails.
    EXPECT_NE(f.find("format=I420"), std::string::npos);
}
TEST(EncoderFragment, X264NoForcedI420) {
    // x264's baseline output caps already constrain the input to I420, so we don't
    // add a redundant capsfilter (keeps the working H.264 pipeline unchanged).
    std::string f = encoder_fragment("x264", "h264", "500");
    EXPECT_EQ(f.find("format=I420"), std::string::npos);
}
TEST(EncoderFragment, H265Nvenc) {
    std::string f = encoder_fragment("nvenc", "hevc", "500");
    EXPECT_NE(f.find("nvh265enc"), std::string::npos);
    EXPECT_NE(f.find("video/x-h265"), std::string::npos);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
