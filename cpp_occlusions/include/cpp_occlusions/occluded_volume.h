#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polyhedron_3.h>

#ifndef REACHABILITY_CORE_H
#define REACHABILITY_CORE_H

typedef CGAL::Simple_cartesian<float>       Kernel;
typedef CGAL::Polygon_2<Kernel>             Polygon;
typedef CGAL::Polyhedron_3<Kernel>          Polyhedron;

namespace cpp_occlusions {

struct ReachabilityParams {
    float vmin = 0;
    float vmax;
    float amin;
    float amax;

    float dt;
    int prediction_horizon;
    float min_shadow_area = 1;
};

class OccludedVolume {

    private:        

        Polyhedron _shadow_polyhedron;
        Polyhedron _road_polyhedron;

        /*
        /// Project a polyhedron onto xy space (might already be in polyhedron_3)
        /// @param polyhedron Polyhedron to project
        /// @return Projected polyhedron
        Polygon _project_xy(Polyhedron polyhedron);
        */

    public:

        OccludedVolume(Polyhedron initial_polyhedron, Polygon road_polygon);

        ~OccludedVolume();

        /// Propagate the reachable set
        /// @param dt Time in s with which the set is propagated
        /// @param polyhedron The polyhedron to propagate. If not specified, 
        /// self._shadow_polyhedron will be propagated
        void Propagate(float dt);
        void Propagate(float dt, Polyhedron &polyhedron);

        /// Generate occupancies for future time intervals
        /// @param dt Time interval for which each occupancy is computed
        /// @param prediction_horizon Number of time steps into the future you want to
        /// generate the occupancies for
        /// @return Array of Polygons for the occupancy within each time interval
        std::vector<Polygon> ComputeFutureOccupancies(float dt, int prediction_horizon);
    };

}

#endif