/*
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "catch.hpp"

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
// Include local header files.
#include "orbmap.hpp"
#include "orbkeypoint.hpp"
#include "orbframe.hpp"
#include <opencv2/core/core.hpp>
TEST_CASE("testConstructor")
{
    opendlv::logic::sensation::OrbMap orbMap();
}
TEST_CASE("testAddOrbKeyFrame")
{
    opendlv::logic::sensation::OrbMap orbMap;
    cv::Mat left, right, tcw;
    left = cv::Mat::zeros(1000, 1000, CV_32F);
    right = cv::Mat::zeros(1000, 1000, CV_32F);
    tcw = cv::Mat::zeros(1000, 1000, CV_32F);
    std::vector<opendlv::logic::sensation::OrbKeyPoint> keyPoints{};
    opendlv::logic::sensation::OrbFrame *frame = new opendlv::logic::sensation::OrbFrame(left, right, keyPoints, tcw);
    frame->Id = 10;
    std::shared_ptr<opendlv::logic::sensation::OrbFrame> keyFrame(frame);
    REQUIRE_NOTHROW(orbMap.PushOrbKeyFrame(keyFrame));
}
TEST_CASE("testMaxKeyFrameId")
{
    opendlv::logic::sensation::OrbMap orbMap;
    cv::Mat left, right, tcw;
    left = cv::Mat::zeros(1000, 1000, CV_32F);
    right = cv::Mat::zeros(1000, 1000, CV_32F);
    tcw = cv::Mat::zeros(1000, 1000, CV_32F);
    std::vector<opendlv::logic::sensation::OrbKeyPoint> keyPoints{};
    opendlv::logic::sensation::OrbFrame *frame = new opendlv::logic::sensation::OrbFrame(left, right, keyPoints, tcw);
    frame->Id = 10;
    std::shared_ptr<opendlv::logic::sensation::OrbFrame> keyFrame(frame);
    REQUIRE_NOTHROW(orbMap.PushOrbKeyFrame(keyFrame));
    REQUIRE(orbMap.MaxKeyFrameId() ==  10);
}
TEST_CASE("testIncrementMajorChangeIndex")
{
    opendlv::logic::sensation::OrbMap orbMap;
    REQUIRE_NOTHROW(orbMap.IncrementMajorChangeIndex());
}
TEST_CASE("testLastMajorChangeIndex")
{
    opendlv::logic::sensation::OrbMap orbMap;
    REQUIRE_NOTHROW(orbMap.IncrementMajorChangeIndex());
    REQUIRE(orbMap.LastMajorChangeIndex() ==  1);
}
TEST_CASE("testAddOrbMapPoint")
{
    opendlv::logic::sensation::OrbMap orbMap;
    std::shared_ptr<opendlv::logic::sensation::OrbMapPoint> mapPoint;
    REQUIRE_NOTHROW(orbMap.PushOrbMapPoint(mapPoint));
}
TEST_CASE("testSetReferenceMapPoints")
{
    opendlv::logic::sensation::OrbMap orbMap;
    std::shared_ptr<opendlv::logic::sensation::OrbMapPoint> mapPoint;
    std::vector<std::shared_ptr<opendlv::logic::sensation::OrbMapPoint>> referenceMapPoints;
    referenceMapPoints.push_back(mapPoint);
    REQUIRE_NOTHROW(orbMap.SetReferenceMapPoints(referenceMapPoints));
}
TEST_CASE("testOrbKeyFramesCount")
{
    opendlv::logic::sensation::OrbMap orbMap;
    cv::Mat left, right, tcw;
    left = cv::Mat::zeros(1000, 1000, CV_32F);
    right = cv::Mat::zeros(1000, 1000, CV_32F);
    tcw = cv::Mat::zeros(1000, 1000, CV_32F);
    std::vector<opendlv::logic::sensation::OrbKeyPoint> keyPoints{};
    opendlv::logic::sensation::OrbFrame *frame = new opendlv::logic::sensation::OrbFrame(left, right, keyPoints, tcw);
    frame->Id = 10;
    std::shared_ptr<opendlv::logic::sensation::OrbFrame> keyFrame(frame);
    REQUIRE_NOTHROW(orbMap.PushOrbKeyFrame(keyFrame));
    REQUIRE(orbMap.OrbKeyFramesCount() ==  1);
}
TEST_CASE("testOrbMapPointsCount")
{
    opendlv::logic::sensation::OrbMap orbMap;
    std::shared_ptr<opendlv::logic::sensation::OrbMapPoint> mapPoint;
    REQUIRE_NOTHROW(orbMap.PushOrbMapPoint(mapPoint));
    REQUIRE(orbMap.OrbMapPointsCount() ==  1);
}
TEST_CASE("testDeleteOrbKeyFrame")
{
    opendlv::logic::sensation::OrbMap orbMap;
    cv::Mat left, right, tcw;
    left = cv::Mat::zeros(1000, 1000, CV_32F);
    right = cv::Mat::zeros(1000, 1000, CV_32F);
    tcw = cv::Mat::zeros(1000, 1000, CV_32F);
    std::vector<opendlv::logic::sensation::OrbKeyPoint> keyPoints{};
    opendlv::logic::sensation::OrbFrame *frame = new opendlv::logic::sensation::OrbFrame(left, right, keyPoints, tcw);
    frame->Id = 10;
    std::shared_ptr<opendlv::logic::sensation::OrbFrame> keyFrame(frame);
    REQUIRE_NOTHROW(orbMap.PushOrbKeyFrame(keyFrame));
    REQUIRE(orbMap.OrbKeyFramesCount() ==  1);
    REQUIRE_NOTHROW(orbMap.DeleteOrbKeyFrame(keyFrame));
    REQUIRE(orbMap.OrbKeyFramesCount() ==  0);
}
TEST_CASE("testDeleteOrbMapPoint")
{
    opendlv::logic::sensation::OrbMap orbMap;
    std::shared_ptr<opendlv::logic::sensation::OrbMapPoint> mapPoint;
    REQUIRE_NOTHROW(orbMap.PushOrbMapPoint(mapPoint));
    REQUIRE(orbMap.OrbMapPointsCount() ==  1);
    REQUIRE_NOTHROW(orbMap.DeleteOrbMapPoint(mapPoint));
    REQUIRE(orbMap.OrbMapPointsCount() ==  0);
}
TEST_CASE("testReset")
{
    opendlv::logic::sensation::OrbMap orbMap;
    std::shared_ptr<opendlv::logic::sensation::OrbMapPoint> mapPoint;
    REQUIRE_NOTHROW(orbMap.PushOrbMapPoint(mapPoint));
    REQUIRE(orbMap.OrbMapPointsCount() ==  1);

    cv::Mat left, right, tcw;
    left = cv::Mat::zeros(1000, 1000, CV_32F);
    right = cv::Mat::zeros(1000, 1000, CV_32F);
    tcw = cv::Mat::zeros(1000, 1000, CV_32F);
    std::vector<opendlv::logic::sensation::OrbKeyPoint> keyPoints{};
    opendlv::logic::sensation::OrbFrame *frame = new opendlv::logic::sensation::OrbFrame(left, right, keyPoints, tcw);
    frame->Id = 10;
    std::shared_ptr<opendlv::logic::sensation::OrbFrame> keyFrame(frame);
    REQUIRE_NOTHROW(orbMap.PushOrbKeyFrame(keyFrame));
    REQUIRE(orbMap.OrbKeyFramesCount() ==  1);

    std::vector<std::shared_ptr<opendlv::logic::sensation::OrbMapPoint>> referenceMapPoints;
    referenceMapPoints.push_back(mapPoint);
    REQUIRE_NOTHROW(orbMap.SetReferenceMapPoints(referenceMapPoints));

    orbMap.Reset();
    REQUIRE(orbMap.OrbMapPointsCount() ==  0);
    REQUIRE(orbMap.OrbKeyFramesCount() ==  0);
}