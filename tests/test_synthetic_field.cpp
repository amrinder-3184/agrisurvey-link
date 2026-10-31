// Tests for include/asl/synthetic_field.h (SYNTHETIC NDVI; added in phase 3).
#include <gtest/gtest.h>

#include "asl/synthetic_field.h"

TEST(SyntheticField, StressedPatchIsLowHealthyIsHigh) {
    EXPECT_NEAR(asl::ndvi_synthetic(asl::kStressCentreEastM, asl::kStressCentreNorthM),
                asl::kStressNdvi, 1e-12);
    EXPECT_GT(asl::ndvi_synthetic(40.0, 40.0), 0.65);   // far from the patch
    EXPECT_GT(asl::ndvi_synthetic(140.0, 100.0), 0.65);
}

TEST(SyntheticField, DeterministicAndInRange) {
    for (double e = 0.0; e <= 180.0; e += 7.5) {
        for (double n = 0.0; n <= 140.0; n += 7.5) {
            const double v = asl::ndvi_synthetic(e, n);
            EXPECT_EQ(v, asl::ndvi_synthetic(e, n));
            EXPECT_GE(v, 0.0);
            EXPECT_LE(v, 1.0);
        }
    }
}

TEST(SyntheticField, ContinuousAtPatchEdge) {
    const double r = asl::kStressRadiusM;
    const double inside = asl::ndvi_synthetic(asl::kStressCentreEastM + r - 1e-6, asl::kStressCentreNorthM);
    const double outside = asl::ndvi_synthetic(asl::kStressCentreEastM + r + 1e-6, asl::kStressCentreNorthM);
    EXPECT_NEAR(inside, outside, 1e-4);
}
