#include "ArmControlPanel.h"

ArmControlPanel::ArmControlPanel(Arm* arm, QWidget* parent) :
    QWidget(parent),
    arm(arm)
{
    QGridLayout* layout = new QGridLayout(this);

    for(int i= 0; i < arm->get_num_joints(); i++)
    {
        QLabel* label =  new QLabel(tr("joint"), this);
        QDoubleSpinBox* box = new QDoubleSpinBox(this);
        box->setRange(arm->get_joint_min(i),
                      arm->get_joint_max(i));
        box->setSingleStep(1.f);
        box->setDecimals(2);
        layout->addWidget(label, i, 0);
        layout->addWidget(box, i, 1);
        connect(box, SIGNAL(valueChanged(double)), this,
                SLOT(updateArm()));
        jointMap[i] = box;
    }
}

void ArmControlPanel::synchronize()
{
    pose values = arm->get_joints();
    for (int i = 0; i < values.size(); i++)
    {
        jointMap[i]->setValue(values.at(i));
    }
}

void ArmControlPanel::updateArm()
{
    pose angles;
    for(int i = 0; i < arm->get_num_joints(); i++)
    {
        angles.push_back(float(jointMap[i]->value()));
    }
    if (arm_state(angles).valid())
    {
        arm->set_joints(angles);
        emit(redrawArm());
    }
    else
    {
        synchronize();
    }
}
