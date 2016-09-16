#define BOOST_TEST_MODULE ConverterTests

#include <boost/test/unit_test.hpp>
#include <data_converter.h>
#include <iostream>

BOOST_AUTO_TEST_CASE(error)
{
	Json::Value root;
    root["beacons"] = "ERR";
	BOOST_CHECK_EQUAL(root, basestation::DataConverter::extractJson("ERR\nDONE"));
}


BOOST_AUTO_TEST_CASE(empty)
{
	Json::Value root;
    root["beacons"] = "";

	BOOST_CHECK_EQUAL(root, basestation::DataConverter::extractJson("NONE"));
	BOOST_CHECK_EQUAL(root, basestation::DataConverter::extractJson(""));
}


BOOST_AUTO_TEST_CASE(well_formed)
{
	Json::Value root;
	Json::Value vector(Json::arrayValue);

	Json::Value beacon;
	beacon["id_tag"] = "2022";
	beacon["rssid"] = "27";
	vector.append(beacon);
	root["beacons"] = vector;

	BOOST_CHECK_EQUAL(root, basestation::DataConverter::extractJson("FOUND,2022,1,27\nDONE"));

	beacon["id_tag"] = "2022";
	beacon["rssid"] = "25";
	vector.append(beacon);
	root["beacons"] = vector;

	BOOST_CHECK_EQUAL(root, basestation::DataConverter::extractJson("FOUND,2022,1,27\nFOUND,2022,1,25\nDONE"));

}

BOOST_AUTO_TEST_CASE(extraction)
{
	Json::Value root;
	Json::Value vector(Json::arrayValue);

	Json::Value beacon;
	beacon["id_tag"] = "2022";
	beacon["rssid"] = "27";
	vector.append(beacon);
	root["beacons"] = vector;

	Json::Value result = basestation::DataConverter::extractJson("FOUND,2022,1,27\nDONE");

	BOOST_CHECK_EQUAL(root["beacons"], result["beacons"]);

	BOOST_CHECK_EQUAL(root["beacons"][0]["rssid"], result["beacons"][0]["rssid"]);



}
