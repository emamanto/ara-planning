#include "ArmControlPanel.h"

ArmControlPanel::ArmControlPanel(Arm* arm, QWidget* parent) :
    QWidget(parent),
    arm(arm)
{
    QGridLayout* layout = new QGridLayout(this);

    int i;
    for(i= 0; i < arm->get_num_joints(); i++)
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

    QPushButton* reset = new QPushButton(tr("reset"), this);
    layout->addWidget(reset, i, 0);
    connect(reset, SIGNAL(clicked()), this, SLOT(resetArm()));
}

void ArmControlPanel::synchronize()
{
    pose values = arm->get_joints();
    for (int i = 0; i < values.size(); i++)
    {
        jointMap[i]->setValue(values.at(i));
    }
}

void ArmControlPanel::disable()
{
    for (int i = 0; i < arm->get_num_joints(); i++)
    {
        jointMap[i]->setEnabled(false);
    }
}

void ArmControlPanel::enable()
{
    for (int i = 0; i < arm->get_num_joints(); i++)
    {
        jointMap[i]->setEnabled(true);
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

void ArmControlPanel::resetArm()
{
    pose angles(arm->get_num_joints(), 0.f);
    arm->set_joints(angles);
    synchronize();
    emit(redrawArm());
}
