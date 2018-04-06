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

#include "cluon-complete.hpp"
#include "selflocalization.hpp"


int32_t main(int32_t argc, char **argv)
{
    std::cout << "Making slammer" << std::endl;
    int32_t retCode{0};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if (0 == commandlineArguments.count("cid"))
    {
        std::cerr << argv[0] << " is a slam implementation for the Chalmers Revere lab." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> [--id=<Identifier in case of simulated units>] [--verbose]" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=111" << std::endl;
        retCode = 1;
    }
    else
    {
        uint32_t const ID
        {
            (commandlineArguments["id"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["id"])) : 0
        };
        bool const VERBOSE
        {
            commandlineArguments.count("verbose") != 0
        };
        (void)VERBOSE;

        // Interface to a running OpenDaVINCI session (ignoring any incoming Envelopes).
        cluon::data::Envelope data;
        std::cout << "Making slammer" << std::endl;
        Selflocalization selflocalization(commandlineArguments);
        cluon::OD4Session od4
        {
            static_cast<uint16_t>(std::stoi(commandlineArguments["cid"])),
            [&data, &orbObject = selflocalization ,&od4session = od4, senderStamp = ID](cluon::data::Envelope &&envelope){
                orbObject.nextContainer(envelope);
            }
        };

        // Just sleep as this microservice is data driven.
        using namespace std::literals::chrono_literals;
        while (od4.isRunning())
        {
            std::this_thread::sleep_for(1s);
            selflocalization.nextContainer(data);
            std::chrono::system_clock::time_point tp;
        }
    }
    return retCode;
}




