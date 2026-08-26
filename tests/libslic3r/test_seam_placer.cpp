#include <catch2/catch_test_macros.hpp>
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
