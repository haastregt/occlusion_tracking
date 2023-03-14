#include <string>
#include <CGAL/Simple_cartesian.h>

#ifndef REACHABILITY_CORE_H
#define REACHABILITY_CORE_H

namespace vehicles {

class Motorcycle {

private:

    /// Name
    std::string _name;

public:

    /// Constructor
    Motorcycle(std::string name);

    /// Get name
    /// @return Name
    std::string get_name() const;

    /// Ride the bike
    /// @param road Name of the road
    void ride(std::string road) const;
};

}

#endif