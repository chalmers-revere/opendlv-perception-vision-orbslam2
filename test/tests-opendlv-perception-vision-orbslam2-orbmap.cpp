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
    TS_ASSERT_THROWS_NOTHING(orbMap.PushOrbKeyFrame(keyFrame));
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
    TS_ASSERT_THROWS_NOTHING(orbMap.PushOrbKeyFrame(keyFrame));
    TS_ASSERT_EQUALS(orbMap.MaxKeyFrameId(), 10);
}
TEST_CASE("testIncrementMajorChangeIndex")
{
    opendlv::logic::sensation::OrbMap orbMap;
    TS_ASSERT_THROWS_NOTHING(orbMap.IncrementMajorChangeIndex());
}
TEST_CASE("testLastMajorChangeIndex")
{
    opendlv::logic::sensation::OrbMap orbMap;
    TS_ASSERT_THROWS_NOTHING(orbMap.IncrementMajorChangeIndex());
    TS_ASSERT_EQUALS(orbMap.LastMajorChangeIndex(), 1);
}
TEST_CASE("testAddOrbMapPoint")
{
    opendlv::logic::sensation::OrbMap orbMap;
    std::shared_ptr<opendlv::logic::sensation::OrbMapPoint> mapPoint;
    TS_ASSERT_THROWS_NOTHING(orbMap.PushOrbMapPoint(mapPoint));
}
TEST_CASE("testSetReferenceMapPoints")
{
    opendlv::logic::sensation::OrbMap orbMap;
    std::shared_ptr<opendlv::logic::sensation::OrbMapPoint> mapPoint;
    std::vector<std::shared_ptr<opendlv::logic::sensation::OrbMapPoint>> referenceMapPoints;
    referenceMapPoints.push_back(mapPoint);
    TS_ASSERT_THROWS_NOTHING(orbMap.SetReferenceMapPoints(referenceMapPoints));
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
    TS_ASSERT_THROWS_NOTHING(orbMap.PushOrbKeyFrame(keyFrame));
    TS_ASSERT_EQUALS(orbMap.OrbKeyFramesCount(), 1);
}
TEST_CASE("testOrbMapPointsCount")
{
    opendlv::logic::sensation::OrbMap orbMap;
    std::shared_ptr<opendlv::logic::sensation::OrbMapPoint> mapPoint;
    TS_ASSERT_THROWS_NOTHING(orbMap.PushOrbMapPoint(mapPoint));
    TS_ASSERT_EQUALS(orbMap.OrbMapPointsCount(), 1);
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
    TS_ASSERT_THROWS_NOTHING(orbMap.PushOrbKeyFrame(keyFrame));
    TS_ASSERT_EQUALS(orbMap.OrbKeyFramesCount(), 1);
    TS_ASSERT_THROWS_NOTHING(orbMap.DeleteOrbKeyFrame(keyFrame));
    TS_ASSERT_EQUALS(orbMap.OrbKeyFramesCount(), 0);
}
TEST_CASE("testDeleteOrbMapPoint")
{
    opendlv::logic::sensation::OrbMap orbMap;
    std::shared_ptr<opendlv::logic::sensation::OrbMapPoint> mapPoint;
    TS_ASSERT_THROWS_NOTHING(orbMap.PushOrbMapPoint(mapPoint));
    TS_ASSERT_EQUALS(orbMap.OrbMapPointsCount(), 1);
    TS_ASSERT_THROWS_NOTHING(orbMap.DeleteOrbMapPoint(mapPoint));
    TS_ASSERT_EQUALS(orbMap.OrbMapPointsCount(), 0);
}
TEST_CASE("testReset")
{
    opendlv::logic::sensation::OrbMap orbMap;
    std::shared_ptr<opendlv::logic::sensation::OrbMapPoint> mapPoint;
    TS_ASSERT_THROWS_NOTHING(orbMap.PushOrbMapPoint(mapPoint));
    TS_ASSERT_EQUALS(orbMap.OrbMapPointsCount(), 1);

    cv::Mat left, right, tcw;
    left = cv::Mat::zeros(1000, 1000, CV_32F);
    right = cv::Mat::zeros(1000, 1000, CV_32F);
    tcw = cv::Mat::zeros(1000, 1000, CV_32F);
    std::vector<opendlv::logic::sensation::OrbKeyPoint> keyPoints{};
    opendlv::logic::sensation::OrbFrame *frame = new opendlv::logic::sensation::OrbFrame(left, right, keyPoints, tcw);
    frame->Id = 10;
    std::shared_ptr<opendlv::logic::sensation::OrbFrame> keyFrame(frame);
    TS_ASSERT_THROWS_NOTHING(orbMap.PushOrbKeyFrame(keyFrame));
    TS_ASSERT_EQUALS(orbMap.OrbKeyFramesCount(), 1);

    std::vector<std::shared_ptr<opendlv::logic::sensation::OrbMapPoint>> referenceMapPoints;
    referenceMapPoints.push_back(mapPoint);
    TS_ASSERT_THROWS_NOTHING(orbMap.SetReferenceMapPoints(referenceMapPoints));

    orbMap.Reset();
    TS_ASSERT_EQUALS(orbMap.OrbMapPointsCount(), 0);
    TS_ASSERT_EQUALS(orbMap.OrbKeyFramesCount(), 0);
}