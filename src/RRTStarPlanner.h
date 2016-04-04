#include "ProbCogSearchStates.h"
#include <ompl/geometric/SimpleSetup.h>

namespace rrtstar
{

class FetchMotionValidator : public ompl::base::MotionValidator
{
public:
    FetchMotionValidator(ompl::base::SpaceInformationPtr& sip);

    bool checkMotion(const ompl::base::State* s1,
                     const ompl::base::State* s2) const;
    bool checkMotion(const ompl::base::State* s1,
                     const ompl::base::State* s2,
                     std::pair<ompl::base::State *, double> &lastValid) const;
};

std::vector<pose> plan(pose b, pose e, float time_limit = 1.0);
}
