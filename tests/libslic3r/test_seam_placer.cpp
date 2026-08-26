#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/GCode/SeamPlacer.hpp"

using namespace Slic3r;
using namespace Catch::Matchers;

namespace {

void add_perimeter(PrintObjectSeamData::LayerSeams &layer, const std::vector<Vec3f> &positions,
                   size_t seam_index, float flow_width, bool finalized = false,
                   const Vec3f &final_position = Vec3f::Zero())
{
    layer.perimeters.emplace_back();
    SeamPlacerImpl::Perimeter &perimeter = layer.perimeters.back();
    perimeter.start_index = layer.points.size();
    perimeter.seam_index = perimeter.start_index + seam_index;
    perimeter.flow_width = flow_width;
    perimeter.finalized = finalized;
    perimeter.final_seam_position = final_position;
    for (const Vec3f &position : positions) {
        layer.points.emplace_back(position, perimeter, 0.0f,
                                  SeamPlacerImpl::EnforcedBlockedSeamPoint::Neutral);
    }
    perimeter.end_index = layer.points.size();
}

} // namespace

TEST_CASE("Aligned seams fill an isolated unaligned layer", "[SeamPlacer][Regression]")
{
    std::vector<PrintObjectSeamData::LayerSeams> layers(3);
    add_perimeter(layers[0], {Vec3f(0.0f, 0.0f, 0.0f)}, 0, 0.23f, true,
                  Vec3f(0.0f, 0.0f, 0.0f));
    add_perimeter(layers[1], {Vec3f(0.6f, 0.0f, 1.0f), Vec3f(0.1f, 0.0f, 1.0f)}, 0, 0.23f);
    add_perimeter(layers[2], {Vec3f(0.0f, 0.0f, 2.0f)}, 0, 0.23f, true,
                  Vec3f(0.0f, 0.0f, 2.0f));

    REQUIRE(SeamPlacerImpl::propagate_seam_alignment(layers, spAligned) == 1);
    const SeamPlacerImpl::Perimeter &perimeter = layers[1].perimeters.front();
    REQUIRE(perimeter.finalized);
    CHECK(perimeter.seam_index == 1);
    CHECK_THAT(perimeter.final_seam_position.x(), WithinAbs(0.0f, 1e-6f));
    CHECK_THAT(perimeter.final_seam_position.y(), WithinAbs(0.0f, 1e-6f));
    CHECK_THAT(perimeter.final_seam_position.z(), WithinAbs(1.0f, 1e-6f));
}

TEST_CASE("Aligned seams propagate through multi-layer gaps", "[SeamPlacer]")
{
    std::vector<PrintObjectSeamData::LayerSeams> layers(4);
    add_perimeter(layers[0], {Vec3f(0.0f, 0.0f, 0.0f)}, 0, 0.23f, true,
                  Vec3f(0.0f, 0.0f, 0.0f));
    add_perimeter(layers[1], {Vec3f(0.1f, 0.0f, 1.0f)}, 0, 0.23f);
    add_perimeter(layers[2], {Vec3f(0.1f, 0.0f, 2.0f)}, 0, 0.23f);
    add_perimeter(layers[3], {Vec3f(0.0f, 0.0f, 3.0f)}, 0, 0.23f, true,
                  Vec3f(0.0f, 0.0f, 3.0f));

    CHECK(SeamPlacerImpl::propagate_seam_alignment(layers, spAligned) == 2);
    CHECK(layers[1].perimeters.front().finalized);
    CHECK(layers[2].perimeters.front().finalized);
}

TEST_CASE("Aligned seams keep isolated candidates outside the alignment distance", "[SeamPlacer]")
{
    std::vector<PrintObjectSeamData::LayerSeams> layers(3);
    add_perimeter(layers[0], {Vec3f(0.0f, 0.0f, 0.0f)}, 0, 0.23f, true,
                  Vec3f(0.0f, 0.0f, 0.0f));
    add_perimeter(layers[1], {Vec3f(2.0f, 0.0f, 1.0f)}, 0, 0.23f);
    add_perimeter(layers[2], {Vec3f(0.0f, 0.0f, 2.0f)}, 0, 0.23f, true,
                  Vec3f(0.0f, 0.0f, 2.0f));

    CHECK(SeamPlacerImpl::propagate_seam_alignment(layers, spAligned) == 0);
    CHECK_FALSE(layers[1].perimeters.front().finalized);
}

TEST_CASE("Aligned seams propagate to the first layers", "[SeamPlacer][Regression]")
{
    std::vector<PrintObjectSeamData::LayerSeams> layers(3);
    add_perimeter(layers[0], {Vec3f(5.0f, 0.0f, 0.0f), Vec3f(0.2f, 0.0f, 0.0f)}, 0, 0.23f);
    add_perimeter(layers[1], {Vec3f(5.0f, 0.0f, 1.0f), Vec3f(0.1f, 0.0f, 1.0f)}, 0, 0.23f);
    add_perimeter(layers[2], {Vec3f(0.0f, 0.0f, 2.0f)}, 0, 0.23f, true,
                  Vec3f(0.0f, 0.0f, 2.0f));

    REQUIRE(SeamPlacerImpl::propagate_seam_alignment(layers, spAligned) == 2);
    CHECK(layers[0].perimeters.front().seam_index == 1);
    CHECK(layers[1].perimeters.front().seam_index == 1);
    CHECK(layers[0].perimeters.front().finalized);
    CHECK(layers[1].perimeters.front().finalized);
}

TEST_CASE("Aligned seams preserve seam blockers while propagating", "[SeamPlacer]")
{
    std::vector<PrintObjectSeamData::LayerSeams> layers(2);
    add_perimeter(layers[0], {Vec3f(5.0f, 0.0f, 0.0f), Vec3f(0.1f, 0.0f, 0.0f)}, 0, 0.23f);
    layers[0].points[1].type = SeamPlacerImpl::EnforcedBlockedSeamPoint::Blocked;
    add_perimeter(layers[1], {Vec3f(0.0f, 0.0f, 1.0f)}, 0, 0.23f, true,
                  Vec3f(0.0f, 0.0f, 1.0f));

    CHECK(SeamPlacerImpl::propagate_seam_alignment(layers, spAligned) == 0);
    CHECK_FALSE(layers[0].perimeters.front().finalized);
}

TEST_CASE("Aligned seams interpolate using variable layer heights", "[SeamPlacer]")
{
    std::vector<PrintObjectSeamData::LayerSeams> layers(3);
    add_perimeter(layers[0], {Vec3f(0.0f, 0.0f, 0.0f)}, 0, 0.23f, true,
                  Vec3f(0.0f, 0.0f, 0.0f));
    add_perimeter(layers[1], {Vec3f(0.1f, 0.0f, 1.0f)}, 0, 0.23f);
    add_perimeter(layers[2], {Vec3f(0.0f, 0.0f, 3.0f)}, 0, 0.23f, true,
                  Vec3f(3.0f, 0.0f, 3.0f));

    REQUIRE(SeamPlacerImpl::propagate_seam_alignment(layers, spAligned) == 1);
    CHECK_THAT(layers[1].perimeters.front().final_seam_position.x(), WithinAbs(1.0f, 1e-6f));
    CHECK_THAT(layers[1].perimeters.front().final_seam_position.z(), WithinAbs(1.0f, 1e-6f));
}

TEST_CASE("Propagated seams follow a moving contour", "[SeamPlacer][Regression]")
{
    const SeamPosition setup = GENERATE(spAligned, spAlignedBack, spRear);
    const bool reverse = GENERATE(false, true);
    constexpr size_t layer_count = 12;
    constexpr float layer_height = 0.2f;
    constexpr float shift = 0.5f;
    constexpr float fitted_offset = 0.1f;
    std::vector<PrintObjectSeamData::LayerSeams> layers(layer_count);
    for (size_t step = 0; step < layer_count; ++step) {
        const size_t layer_idx = reverse ? layer_count - 1 - step : step;
        const float z = layer_height * layer_idx;
        add_perimeter(layers[layer_idx], {Vec3f(shift * step, 0, z)}, 0, 0.4f,
                      step == 0, Vec3f(fitted_offset, 0, z));
    }

    REQUIRE(SeamPlacerImpl::propagate_seam_alignment(layers, setup) == layer_count - 1);
    for (const auto &layer : layers) {
        const auto &perimeter = layer.perimeters.front();
        CHECK_THAT(perimeter.final_seam_position.x(),
                   WithinAbs(layer.points[perimeter.seam_index].position.x() + fitted_offset, 1e-5f));
    }
}

TEST_CASE("Propagated seams discard a distant fitted target", "[SeamPlacer][Regression]")
{
    const bool two_anchors = GENERATE(false, true);
    std::vector<PrintObjectSeamData::LayerSeams> layers(two_anchors ? 3 : 2);
    add_perimeter(layers[0], {Vec3f(0, 0, 0)}, 0, 0.4f, true, Vec3f(10, 0, 0));
    add_perimeter(layers[1], {Vec3f(0.1f, 0, 1)}, 0, 0.4f);
    if (two_anchors)
        add_perimeter(layers[2], {Vec3f(0, 0, 2)}, 0, 0.4f, true, Vec3f(10, 0, 2));

    REQUIRE(SeamPlacerImpl::propagate_seam_alignment(layers, spAligned) == 1);
    CHECK_THAT(layers[1].perimeters.front().final_seam_position.x(), WithinAbs(0.1f, 1e-6f));
}

TEST_CASE("Propagated seams keep the final position out of painted restrictions", "[SeamPlacer][Regression]")
{
    const SeamPosition setup = GENERATE(spAligned, spAlignedBack, spRear);
    const auto candidate_type = GENERATE(SeamPlacerImpl::EnforcedBlockedSeamPoint::Neutral,
                                         SeamPlacerImpl::EnforcedBlockedSeamPoint::Enforced);
    std::vector<PrintObjectSeamData::LayerSeams> layers(3);
    add_perimeter(layers[0], {Vec3f(0, 0, 0)}, 0, 0.4f, true, Vec3f(0, 0, 0));
    add_perimeter(layers[1], {Vec3f(5, 0, 1), Vec3f(0.2f, 0, 1), Vec3f(0, 0, 1)}, 0, 0.4f);
    layers[1].points[1].type = candidate_type;
    layers[1].points[2].type = SeamPlacerImpl::EnforcedBlockedSeamPoint::Blocked;
    add_perimeter(layers[2], {Vec3f(0, 0, 2)}, 0, 0.4f, true, Vec3f(0, 0, 2));

    REQUIRE(SeamPlacerImpl::propagate_seam_alignment(layers, setup) == 1);
    CHECK(layers[1].perimeters.front().seam_index == 1);
    CHECK_THAT(layers[1].perimeters.front().final_seam_position.x(), WithinAbs(0.2f, 1e-6f));
}

TEST_CASE("Propagated seams preserve selected enforcers", "[SeamPlacer][Regression]")
{
    const SeamPosition setup = GENERATE(spAligned, spAlignedBack, spRear);
    const bool central = GENERATE(false, true);
    std::vector<PrintObjectSeamData::LayerSeams> layers(2);
    add_perimeter(layers[0], {Vec3f(5, 0, 0), Vec3f(0.1f, 0, 0)}, 0, 0.4f);
    layers[0].points[0].type = SeamPlacerImpl::EnforcedBlockedSeamPoint::Enforced;
    layers[0].points[0].central_enforcer = central;
    add_perimeter(layers[1], {Vec3f(0, 0, 1)}, 0, 0.4f, true, Vec3f(0, 0, 1));

    CHECK(SeamPlacerImpl::propagate_seam_alignment(layers, setup) == 0);
    CHECK_FALSE(layers[0].perimeters.front().finalized);
}

TEST_CASE("Propagated seams preserve overhang and embedding priorities", "[SeamPlacer][Regression]")
{
    const SeamPosition setup = GENERATE(spAligned, spAlignedBack, spRear);
    const bool overhang = GENERATE(false, true);
    std::vector<PrintObjectSeamData::LayerSeams> layers(3);
    add_perimeter(layers[0], {Vec3f(0, 0, 0)}, 0, 0.4f, true, Vec3f(0, 0, 0));
    add_perimeter(layers[1], {Vec3f(5, 0, 1), Vec3f(0.1f, 0, 1)}, 0, 0.4f);
    if (overhang)
        layers[1].points[1].overhang = 2.0f;
    else
        layers[1].points[0].embedded_distance = -1.0f;
    add_perimeter(layers[2], {Vec3f(0, 0, 2)}, 0, 0.4f, true, Vec3f(0, 0, 2));

    CHECK(SeamPlacerImpl::propagate_seam_alignment(layers, setup) == 0);
    CHECK(layers[1].perimeters.front().seam_index == 0);
    CHECK_FALSE(layers[1].perimeters.front().finalized);
}

TEST_CASE("Back seam propagation preserves the rear preference", "[SeamPlacer][Regression]")
{
    std::vector<PrintObjectSeamData::LayerSeams> layers(3);
    add_perimeter(layers[0], {Vec3f(0, 0, 0)}, 0, 0.4f, true, Vec3f(0, 0, 0));
    add_perimeter(layers[1], {Vec3f(0, 5, 1), Vec3f(0, 0.1f, 1)}, 0, 0.4f);
    add_perimeter(layers[2], {Vec3f(0, 0, 2)}, 0, 0.4f, true, Vec3f(0, 0, 2));

    CHECK(SeamPlacerImpl::propagate_seam_alignment(layers, spRear) == 0);
    CHECK(layers[1].perimeters.front().seam_index == 0);
}

TEST_CASE("Aligned seam propagation tolerates local visibility fluctuations", "[SeamPlacer][Regression]")
{
    const SeamPosition setup = GENERATE(spAligned, spAlignedBack);
    std::vector<PrintObjectSeamData::LayerSeams> layers(3);
    add_perimeter(layers[0], {Vec3f(0, 0, 0)}, 0, 0.4f, true, Vec3f(0, 0, 0));
    add_perimeter(layers[1], {Vec3f(5, 0, 1), Vec3f(0.1f, 0, 1)}, 0, 0.4f);
    layers[1].points[1].visibility = 1.0f;
    add_perimeter(layers[2], {Vec3f(0, 0, 2)}, 0, 0.4f, true, Vec3f(0, 0, 2));

    REQUIRE(SeamPlacerImpl::propagate_seam_alignment(layers, setup) == 1);
    CHECK(layers[1].perimeters.front().seam_index == 1);
    CHECK_THAT(layers[1].perimeters.front().final_seam_position.x(), WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("Propagated seam targets stay on supported contour points", "[SeamPlacer][Regression]")
{
    const SeamPosition setup = GENERATE(spAligned, spAlignedBack, spRear);
    std::vector<PrintObjectSeamData::LayerSeams> layers(3);
    add_perimeter(layers[0], {Vec3f(0, 0, 0)}, 0, 0.4f, true, Vec3f(0, 0, 0));
    add_perimeter(layers[1], {Vec3f(5, 0, 1), Vec3f(0.2f, 0, 1), Vec3f(0, 0, 1)}, 0, 0.4f);
    layers[1].points[2].overhang = 1.0f;
    add_perimeter(layers[2], {Vec3f(0, 0, 2)}, 0, 0.4f, true, Vec3f(0, 0, 2));

    REQUIRE(SeamPlacerImpl::propagate_seam_alignment(layers, setup) == 1);
    CHECK(layers[1].perimeters.front().seam_index == 1);
    CHECK_THAT(layers[1].perimeters.front().final_seam_position.x(), WithinAbs(0.2f, 1e-6f));
}

TEST_CASE("Back seam fitted targets preserve the rear preference", "[SeamPlacer][Regression]")
{
    std::vector<PrintObjectSeamData::LayerSeams> layers(3);
    add_perimeter(layers[0], {Vec3f(0, -0.5f, 0)}, 0, 0.4f, true, Vec3f(0, -1.5f, 0));
    add_perimeter(layers[1], {Vec3f(0, 1, 1), Vec3f(0, 0, 1)}, 0, 0.4f);
    add_perimeter(layers[2], {Vec3f(0, -0.5f, 2)}, 0, 0.4f, true, Vec3f(0, -1.5f, 2));

    REQUIRE(SeamPlacerImpl::propagate_seam_alignment(layers, spRear) == 1);
    CHECK(layers[1].perimeters.front().seam_index == 1);
    CHECK_THAT(layers[1].perimeters.front().final_seam_position.y(), WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("Seam propagation crosses long gaps beside unrelated aligned contours", "[SeamPlacer][Regression]")
{
    constexpr size_t layer_count = 128;
    std::vector<PrintObjectSeamData::LayerSeams> layers(layer_count);
    for (size_t layer_idx = 0; layer_idx < layer_count; ++layer_idx) {
        const float z = 0.2f * layer_idx;
        add_perimeter(layers[layer_idx], {Vec3f(0, 0, z)}, 0, 0.4f,
                      layer_idx == 0, Vec3f(0, 0, z));
        add_perimeter(layers[layer_idx], {Vec3f(100, 0, z)}, 0, 0.4f, true, Vec3f(100, 0, z));
    }

    REQUIRE(SeamPlacerImpl::propagate_seam_alignment(layers, spAligned) == layer_count - 1);
    for (const auto &layer : layers) {
        CHECK(layer.perimeters.front().finalized);
        CHECK_THAT(layer.perimeters.front().final_seam_position.x(), WithinAbs(0.0f, 1e-6f));
        CHECK_THAT(layer.perimeters.back().final_seam_position.x(), WithinAbs(100.0f, 1e-6f));
    }
    CHECK(SeamPlacerImpl::propagate_seam_alignment(layers, spAligned) == 0);
}

TEST_CASE("Seam propagation stops at a blocked layer", "[SeamPlacer][Regression]")
{
    std::vector<PrintObjectSeamData::LayerSeams> layers(5);
    for (size_t layer_idx = 0; layer_idx < layers.size(); ++layer_idx) {
        const float z = 0.2f * layer_idx;
        add_perimeter(layers[layer_idx], {Vec3f(0, 0, z)}, 0, 0.4f,
                      layer_idx == 3, Vec3f(0, 0, z));
    }
    layers[1].points[0].type = SeamPlacerImpl::EnforcedBlockedSeamPoint::Blocked;

    CHECK(SeamPlacerImpl::propagate_seam_alignment(layers, spAligned) == 2);
    CHECK_FALSE(layers[0].perimeters.front().finalized);
    CHECK_FALSE(layers[1].perimeters.front().finalized);
    CHECK(layers[2].perimeters.front().finalized);
    CHECK(layers[4].perimeters.front().finalized);
}

TEST_CASE("Seams extend to initial layers with small support-score increases", "[SeamPlacer][Regression]")
{
    const SeamPosition setup = GENERATE(spAligned, spAlignedBack, spRear);
    std::vector<PrintObjectSeamData::LayerSeams> layers(3);
    add_perimeter(layers[0], {Vec3f(2, -0.1f, 0), Vec3f(2, -3, 0), Vec3f(-2, -3, 0),
                              Vec3f(-2, -0.1f, 0), Vec3f(0, -0.1f, 0)}, 0, 0.25f);
    add_perimeter(layers[1], {Vec3f(2, 0, 0.02f), Vec3f(2, -3, 0.02f), Vec3f(-2, -3, 0.02f),
                              Vec3f(-2, 0, 0.02f), Vec3f(0, 0, 0.02f)}, 0, 0.23f);
    for (auto &point : layers[1].points)
        point.overhang = 0.09f;
    layers[1].points[0].overhang = 0.04f;
    add_perimeter(layers[2], {Vec3f(0, 0, 0.04f)}, 0, 0.23f, true, Vec3f(-0.45f, 0.92f, 0.04f));

    REQUIRE(SeamPlacerImpl::propagate_seam_alignment(layers, setup) == 2);
    CHECK_THAT(layers[0].perimeters.front().final_seam_position.x(), WithinAbs(-0.45f, 1e-5f));
    CHECK_THAT(layers[1].perimeters.front().final_seam_position.x(), WithinAbs(-0.45f, 1e-5f));
}

TEST_CASE("Small overhang scores do not snap an interpolated seam to a vertex", "[SeamPlacer][Regression]")
{
    const SeamPosition setup = GENERATE(spAligned, spAlignedBack, spRear);
    std::vector<PrintObjectSeamData::LayerSeams> layers(3);
    add_perimeter(layers[0], {Vec3f(0, 0, 0)}, 0, 0.23f, true, Vec3f(-0.17f, 0.25f, 0));
    add_perimeter(layers[1], {Vec3f(2, 0, 0.02f), Vec3f(0.01f, 0, 0.02f)}, 0, 0.23f);
    layers[1].points[1].overhang = 0.0015f;
    add_perimeter(layers[2], {Vec3f(0, 0, 0.04f)}, 0, 0.23f, true, Vec3f(-0.17f, 0.25f, 0.04f));

    REQUIRE(SeamPlacerImpl::propagate_seam_alignment(layers, setup) == 1);
    CHECK_THAT(layers[1].perimeters.front().final_seam_position.x(), WithinAbs(-0.17f, 1e-5f));
    CHECK_THAT(layers[1].perimeters.front().final_seam_position.y(), WithinAbs(0.25f, 1e-5f));
}

TEST_CASE("Seam propagation rejects support-score increases beyond half a line width", "[SeamPlacer][Regression]")
{
    const SeamPosition setup = GENERATE(spAligned, spAlignedBack, spRear);
    std::vector<PrintObjectSeamData::LayerSeams> layers(2);
    add_perimeter(layers[0], {Vec3f(2, 0, 0), Vec3f(0, 0, 0)}, 0, 0.23f);
    layers[0].points[1].overhang = 0.12f;
    add_perimeter(layers[1], {Vec3f(0, 0, 0.02f)}, 0, 0.23f, true, Vec3f(0, 0, 0.02f));

    CHECK(SeamPlacerImpl::propagate_seam_alignment(layers, setup) == 0);
    CHECK_FALSE(layers[0].perimeters.front().finalized);
}
