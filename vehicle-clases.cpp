/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If
you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not
required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original
software.
3. This notice may not be removed or altered from any source distribution.
*/

/// September 2006: VehicleDemo is work in progress, this file is mostly just a placeholder
/// This VehicleDemo file is very early in development, please check it later
///@todo is a basic engine model:
/// A function that maps user input (throttle) into torque/force applied on the wheels
/// with gears etc.
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "btBulletDynamicsCommon.h"

#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"

class btVehicleTuning;
struct btVehicleRaycaster;
class btCollisionShape;

#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"
#include "BulletDynamics/Vehicle/btRaycastVehicle.h"

#include "../CommonInterfaces/CommonExampleInterface.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "../CommonInterfaces/CommonWindowInterface.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btBulletCollisionCommon.h"

/// VehicleDemo shows how to setup and use the built-in raycast vehicle
class Hinge2Vehicle : public CommonExampleInterface
{
  public:
    GUIHelperInterface *m_guiHelper;

    /* extra stuff*/
    btVector3 m_cameraPosition;
    class btDiscreteDynamicsWorld *m_dynamicsWorld;
    btDiscreteDynamicsWorld *getDynamicsWorld()
    {
        return m_dynamicsWorld;
    }
    btRigidBody *m_carChassis;
    btRigidBody *localCreateRigidBody(btScalar mass, const btTransform &worldTransform, btCollisionShape *colSape);

    int m_wheelInstances[12];
    int m_wheelInstancesCount = 0;

    //----------------------------

    bool m_useDefaultCamera;
    btVector3 m_cameraTargetPosition;
    //----------------------------

    btAlignedObjectArray<btCollisionShape *> m_collisionShapes;

    class btBroadphaseInterface *m_overlappingPairCache;

    class btCollisionDispatcher *m_dispatcher;

    class btConstraintSolver *m_constraintSolver;

    class btDefaultCollisionConfiguration *m_collisionConfiguration;

    class btTriangleIndexVertexArray *m_indexVertexArrays;

    btVector3 *m_vertices = nullptr;

    btRaycastVehicle::btVehicleTuning m_tuning;
    btVehicleRaycaster *m_vehicleRC = nullptr;
    btRaycastVehicle *m_vehicle = nullptr;
    btCollisionShape *m_wheelShape = nullptr;

    btRaycastVehicle *m_wagon_0 = nullptr;
    btRaycastVehicle *m_wagon_1 = nullptr;

    float m_cameraHeight;

    float m_minCameraDistance;
    float m_maxCameraDistance;

    Hinge2Vehicle(struct GUIHelperInterface *helper);

    virtual ~Hinge2Vehicle();

    virtual void stepSimulation(float deltaTime);

    virtual void resetForklift();

    virtual void clientResetScene();

    virtual void displayCallback();

    virtual void specialKeyboard(int key, int x, int y);

    virtual void specialKeyboardUp(int key, int x, int y);

    virtual bool mouseMoveCallback(float x, float y)
    {
        return false;
    }

    virtual bool mouseButtonCallback(int button, int state, float x, float y)
    {
        return false;
    }

    virtual bool keyboardCallback(int key, int state);

    virtual void renderScene();

    virtual void physicsDebugDraw(int debugFlags);

    void initPhysics();
    void exitPhysics();

    virtual void resetCamera()
    {
        float dist = 8;
        float pitch = -32;
        float yaw = -45;
        float targetPos[3] = {-0.33, -0.72, 4.5};
        m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
    }

    void updateCamera();

    // PRACTICA
    void createTower(btScalar posX, btScalar posZ);
    btRaycastVehicle *createVagon(btRaycastVehicle *parent);
};

btScalar max_MotorImpulse = 4000.f;

// the sequential impulse solver has difficulties dealing with large mass ratios (differences), between loadMass and the
// fork parts btScalar loadMass = 10.f;//this should work fine for the SI solver

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4 0.785398163397448309616
#endif

btVector3 wheelDirection_CS0(0, -1, 0);
btVector3 wheelAxle_CS(-1, 0, 0);

bool use_MCLPSolver = true;

#include <stdio.h> //printf debugging

#include "Hinge2Vehicle.h"

/// btRaycastVehicle is the interface for the constraint that implements the raycast vehicle
/// notice that for higher-quality slow-moving vehicles, another approach might be better
/// implementing explicit hinged-wheel constraints with cylinder collision, rather then raycasts
float g_EngineForce = 0.f;

float default_BreakingForce = 10.f;
float g_BreakingForce = 100.f;

float max_EngineForce = 2000.f; // this should be engine/velocity dependent
float max_BreakingForce = 200.f;

float g_VehicleSteering = 0.f;
float steering_Increment = 0.0f;
float maxsteering_Increment = 0.01f;
float steering_Clamp = 0.3f;
float wheel_Radius = 0.5f;
float wheel_Width = 0.4f;
float wheel_Friction = 1000; // BT_LARGE_FLOAT;
float suspension_Stiffness = 20.f;
float suspension_Damping = 2.3f;
float suspension_Compression = 4.4f;
float roll_Influence = 0.1f; // 1.0f;

btScalar suspension_RestLength(0.6);

#define CUBE_HALF_EXTENTS 1

////////////////////////////////////

#define ARRAY_SIZE_X 3
#define ARRAY_SIZE_Y 12
#define ARRAY_SIZE_Z 3
#define TOWER_NUM 10

#define START_POS_Y 0.5f
#define SCALING 1.0f

static inline btScalar UnitRand()
{
    return (rand() / (btScalar)RAND_MAX);
}
static inline btScalar SignedUnitRand()
{
    return (UnitRand() * 2 - 1);
}

void Hinge2Vehicle::createTower(btScalar posX, btScalar posZ)
{
    btTransform startTransform;
    startTransform.setIdentity();

    float start_x = posX - ARRAY_SIZE_X / 2;
    float start_y = START_POS_Y;
    float start_z = posZ - ARRAY_SIZE_Z / 2;

    btScalar side = 0.5f;
    btScalar sideHalf = side / 2.f;
    btVector3 halfExtents = btVector3(sideHalf, sideHalf, sideHalf);
    btScalar mass = 1.0f;
    btBoxShape *colShape = new btBoxShape(halfExtents);
    btVector3 localInertia(0, 0, 0);
    colShape->calculateLocalInertia(mass, localInertia);

    int i = 0, j = 0, k = 0;

    for (int k = 0; k < ARRAY_SIZE_Y; k++)
        for (int i = 0; i < ARRAY_SIZE_X; i++)
            for (int j = 0; j < ARRAY_SIZE_Z; j++)
            {
                startTransform.setOrigin(
                    btVector3(btScalar(posX + side * i), btScalar(sideHalf + side * k), btScalar(posZ + side * j)));

                btDefaultMotionState *myMotionState = new btDefaultMotionState(startTransform);
                btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, colShape, localInertia);
                btRigidBody *body = new btRigidBody(cInfo);
                body->setActivationState(ISLAND_SLEEPING);

                body->setUserIndex(-1);
                m_dynamicsWorld->addRigidBody(body);
            }
}

btRaycastVehicle *Hinge2Vehicle::createVagon(btRaycastVehicle *parent_vehicle)
{
    if (!parent_vehicle)
        return 0;

    btRigidBody *parentBody = parent_vehicle->getRigidBody(); // get the rigid body of the car

    btVector3 parentPos = parentBody->getCenterOfMassPosition(); // posicion del coche
    btVector3 aabbMin, aabbMax;
    parentBody->getAabb(aabbMin, aabbMax);
    btScalar parentSizeX = (aabbMax.getX() - aabbMin.getX()); // ancho del coche
    btScalar parentSizeY = (aabbMax.getY() - aabbMin.getY()); // altura del coche
    btScalar parentSizeZ = (aabbMax.getZ() - aabbMin.getZ()); // longitud del coche
    btVector3 parentAxis(0.f, 1.f, 0.f);
    btVector3 childAxis(1.f, 0.f, 0.f);

    // Mine

    btCollisionShape *chassisShape = new btBoxShape(btVector3(1.f, 0.5f, 2.f));
    m_collisionShapes.push_back(chassisShape);

    btCompoundShape *compound = new btCompoundShape();
    m_collisionShapes.push_back(compound);

    btTransform tr;
    tr.setIdentity();
    tr.setOrigin(btVector3(0, 1, 0));

    compound->addChildShape(tr, chassisShape);

    {
        btCollisionShape *suppShape = new btBoxShape(btVector3(0.5f, 0.1f, 0.5f));
        btTransform suppLocalTrans;
        suppLocalTrans.setIdentity();
        // localTrans effectively shifts the center of mass with respect to the chassis
        suppLocalTrans.setOrigin(btVector3(0, 1.0, 2.5));
        compound->addChildShape(suppLocalTrans, suppShape);
    }

    tr.setOrigin(parentPos + btVector3(0, 0, -5.75f));

    auto chassis = localCreateRigidBody(800, tr, compound);
    auto wheelShape = new btCylinderShapeX(btVector3(wheel_Width, wheel_Radius, wheel_Radius));

    m_guiHelper->createCollisionShapeGraphicsObject(wheelShape);
    int wheelGraphicsIndex = wheelShape->getUserIndex();

    const float position[4] = {0, 10, 10, 0};
    const float quaternion[4] = {0, 0, 0, 1};
    const float color[4] = {0, 1, 0, 1};
    const float scaling[4] = {1, 1, 1, 1};

    for (int i = 0; i < 4; i++)
    {
        m_wheelInstances[m_wheelInstancesCount++] =
            m_guiHelper->registerGraphicsInstance(wheelGraphicsIndex, position, quaternion, color, scaling);
    }

    auto vehicleRC = new btDefaultVehicleRaycaster(m_dynamicsWorld);
    auto vehicle = new btRaycastVehicle(m_tuning, chassis, vehicleRC);

    chassis->setActivationState(DISABLE_DEACTIVATION);

    m_dynamicsWorld->addVehicle(vehicle);

    float connectionHeight = 1.2f;

    bool isFrontWheel = true;

    // choose coordinate system
    const int rightIndex = 0;
    const int upIndex = 1;
    const int forwardIndex = 2;
    vehicle->setCoordinateSystem(rightIndex, upIndex, forwardIndex);

    btVector3 connectionPointCS0(CUBE_HALF_EXTENTS - (0.3 * wheel_Width), connectionHeight,
                                 2 * CUBE_HALF_EXTENTS - wheel_Radius);

    vehicle->addWheel(connectionPointCS0, wheelDirection_CS0, wheelAxle_CS, suspension_RestLength, wheel_Radius,
                      m_tuning, isFrontWheel);
    connectionPointCS0 =
        btVector3(-CUBE_HALF_EXTENTS + (0.3 * wheel_Width), connectionHeight, 2 * CUBE_HALF_EXTENTS - wheel_Radius);

    vehicle->addWheel(connectionPointCS0, wheelDirection_CS0, wheelAxle_CS, suspension_RestLength, wheel_Radius,
                      m_tuning, isFrontWheel);
    connectionPointCS0 =
        btVector3(-CUBE_HALF_EXTENTS + (0.3 * wheel_Width), connectionHeight, -2 * CUBE_HALF_EXTENTS + wheel_Radius);
    isFrontWheel = false;
    vehicle->addWheel(connectionPointCS0, wheelDirection_CS0, wheelAxle_CS, suspension_RestLength, wheel_Radius,
                      m_tuning, isFrontWheel);
    connectionPointCS0 =
        btVector3(CUBE_HALF_EXTENTS - (0.3 * wheel_Width), connectionHeight, -2 * CUBE_HALF_EXTENTS + wheel_Radius);
    vehicle->addWheel(connectionPointCS0, wheelDirection_CS0, wheelAxle_CS, suspension_RestLength, wheel_Radius,
                      m_tuning, isFrontWheel);

    auto wheelNum = vehicle->getNumWheels();

    for (int i = 0; i < wheelNum; i++)
    {
        btWheelInfo &wheel = vehicle->getWheelInfo(i);
        wheel.m_suspensionStiffness = suspension_Stiffness;
        wheel.m_wheelsDampingRelaxation = suspension_Damping;
        wheel.m_wheelsDampingCompression = suspension_Compression;
        wheel.m_frictionSlip = wheel_Friction;
        wheel.m_rollInfluence = roll_Influence;
    }

    auto anchor = parentPos;
    float a = anchor.getZ();
    anchor.setZ(a - parentSizeZ / 2.f);
    btHinge2Constraint *hinge = new btHinge2Constraint(*parentBody, *chassis, anchor, parentAxis, childAxis);
    m_dynamicsWorld->addConstraint(hinge, true);

    return vehicle;
}

Hinge2Vehicle::Hinge2Vehicle(struct GUIHelperInterface *helper)
    : m_guiHelper(helper), m_carChassis(0), m_indexVertexArrays(0), m_vertices(0), m_cameraHeight(4.f),
      m_minCameraDistance(3.f), m_maxCameraDistance(10.f)
{
    helper->setUpAxis(1);
    m_vehicle = 0;
    m_wheelShape = 0;
    m_cameraPosition = btVector3(30, 30, 30);
    m_useDefaultCamera = false;
    //	setTexturing(true);
    //	setShadows(true);
}

void Hinge2Vehicle::exitPhysics()
{
    // cleanup in the reverse order of creation/initialization

    // remove the rigidbodies from the dynamics world and delete them
    int i;
    for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
    {
        btCollisionObject *obj = m_dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody *body = btRigidBody::upcast(obj);
        if (body && body->getMotionState())
        {
            while (body->getNumConstraintRefs())
            {
                btTypedConstraint *constraint = body->getConstraintRef(0);
                m_dynamicsWorld->removeConstraint(constraint);
                delete constraint;
            }
            delete body->getMotionState();
            m_dynamicsWorld->removeRigidBody(body);
        }
        else
        {
            m_dynamicsWorld->removeCollisionObject(obj);
        }
        delete obj;
    }

    // delete collision shapes
    for (int j = 0; j < m_collisionShapes.size(); j++)
    {
        btCollisionShape *shape = m_collisionShapes[j];
        delete shape;
    }
    m_collisionShapes.clear();

    delete m_indexVertexArrays;
    delete m_vertices;

    // delete dynamics world
    delete m_dynamicsWorld;
    m_dynamicsWorld = 0;

    delete m_vehicleRC;
    m_vehicleRC = 0;

    delete m_vehicle;
    m_vehicle = 0;

    delete m_wagon_0;
    m_wagon_0 = 0;

    delete m_wagon_1;
    m_wagon_1 = 0;

    delete m_wheelShape;
    m_wheelShape = 0;

    // delete solver
    delete m_constraintSolver;
    m_constraintSolver = 0;

    // delete broadphase
    delete m_overlappingPairCache;
    m_overlappingPairCache = 0;

    // delete dispatcher
    delete m_dispatcher;
    m_dispatcher = 0;

    delete m_collisionConfiguration;
    m_collisionConfiguration = 0;
}

Hinge2Vehicle::~Hinge2Vehicle()
{
    // exitPhysics();
}

void Hinge2Vehicle::initPhysics()
{
    int upAxis = 1;

    m_guiHelper->setUpAxis(upAxis);

    btVector3 groundExtents(250, 5, 250);
    groundExtents[upAxis] = 3;
    btCollisionShape *groundShape = new btBoxShape(groundExtents);
    m_collisionShapes.push_back(groundShape);
    m_collisionConfiguration = new btDefaultCollisionConfiguration();
    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
    btVector3 worldMin(-1000, -1000, -1000);
    btVector3 worldMax(1000, 1000, 1000);
    m_overlappingPairCache = new btAxisSweep3(worldMin, worldMax);
    if (0) // use_MCLPSolver)//causes huge slow down when car collides with towers:(
    {
        btDantzigSolver *mlcp = new btDantzigSolver();
        // btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
        btMLCPSolver *sol = new btMLCPSolver(mlcp);
        m_constraintSolver = sol;
    }
    else
    {
        m_constraintSolver = new btSequentialImpulseConstraintSolver();
    }
    m_dynamicsWorld =
        new btDiscreteDynamicsWorld(m_dispatcher, m_overlappingPairCache, m_constraintSolver, m_collisionConfiguration);
    if (use_MCLPSolver)
    {
        m_dynamicsWorld->getSolverInfo().m_minimumSolverBatchSize =
            1; // for direct solver it is better to have a small A matrix
    }
    else
    {
        m_dynamicsWorld->getSolverInfo().m_minimumSolverBatchSize =
            128; // for direct solver, it is better to solve multiple objects together, small batches have high overhead
    }
    m_dynamicsWorld->getSolverInfo().m_globalCfm = 0.00001;

    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

    // m_dynamicsWorld->setGravity(btVector3(0,0,0));
    btTransform tr;
    tr.setIdentity();
    tr.setOrigin(btVector3(0, -3, 0));

    // either use heightfield or triangle mesh

    // create ground object
    localCreateRigidBody(0, tr, groundShape);

    btCollisionShape *chassisShape = new btBoxShape(btVector3(1.f, 0.5f, 2.f));
    m_collisionShapes.push_back(chassisShape);

    btCompoundShape *compound = new btCompoundShape();
    m_collisionShapes.push_back(compound);
    btTransform localTrans;
    localTrans.setIdentity();
    // localTrans effectively shifts the center of mass with respect to the chassis
    localTrans.setOrigin(btVector3(0, 1, 0));

    compound->addChildShape(localTrans, chassisShape);

    {
        btCollisionShape *suppShape = new btBoxShape(btVector3(0.5f, 0.1f, 0.5f));
        btTransform suppLocalTrans;
        suppLocalTrans.setIdentity();
        // localTrans effectively shifts the center of mass with respect to the chassis
        suppLocalTrans.setOrigin(btVector3(0, 1.0, 2.5));
        compound->addChildShape(suppLocalTrans, suppShape);
    }

    tr.setOrigin(btVector3(0, 0.f, 0));

    m_carChassis = localCreateRigidBody(800, tr, compound); // chassisShape);
    // m_carChassis->setDamping(0.2,0.2);

    // only needed to register the wheel in the GUI
    m_wheelShape = new btCylinderShapeX(btVector3(wheel_Width, wheel_Radius, wheel_Radius));

    m_guiHelper->createCollisionShapeGraphicsObject(m_wheelShape);
    int wheelGraphicsIndex = m_wheelShape->getUserIndex();

    const float position[4] = {0, 10, 10, 0};
    const float quaternion[4] = {0, 0, 0, 1};
    const float color[4] = {0, 1, 0, 1};
    const float scaling[4] = {1, 1, 1, 1};

    for (int i = 0; i < 4; i++)
    {
        m_wheelInstances[m_wheelInstancesCount++] =
            m_guiHelper->registerGraphicsInstance(wheelGraphicsIndex, position, quaternion, color, scaling);
    }

    /// create vehicle
    {
        m_vehicleRC = new btDefaultVehicleRaycaster(m_dynamicsWorld);
        m_vehicle = new btRaycastVehicle(m_tuning, m_carChassis, m_vehicleRC);

        /// never deactivate the vehicle
        m_carChassis->setActivationState(DISABLE_DEACTIVATION);

        m_dynamicsWorld->addVehicle(m_vehicle);

        float connectionHeight = 1.2f;

        bool isFrontWheel = true;

        // choose coordinate system
        const int rightIndex = 0;
        const int upIndex = 1;
        const int forwardIndex = 2;
        m_vehicle->setCoordinateSystem(rightIndex, upIndex, forwardIndex);

        btVector3 connectionPointCS0(CUBE_HALF_EXTENTS - (0.3 * wheel_Width), connectionHeight,
                                     2 * CUBE_HALF_EXTENTS - wheel_Radius);

        m_vehicle->addWheel(connectionPointCS0, wheelDirection_CS0, wheelAxle_CS, suspension_RestLength, wheel_Radius,
                            m_tuning, isFrontWheel);
        connectionPointCS0 =
            btVector3(-CUBE_HALF_EXTENTS + (0.3 * wheel_Width), connectionHeight, 2 * CUBE_HALF_EXTENTS - wheel_Radius);

        m_vehicle->addWheel(connectionPointCS0, wheelDirection_CS0, wheelAxle_CS, suspension_RestLength, wheel_Radius,
                            m_tuning, isFrontWheel);
        connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS + (0.3 * wheel_Width), connectionHeight,
                                       -2 * CUBE_HALF_EXTENTS + wheel_Radius);
        isFrontWheel = false;
        m_vehicle->addWheel(connectionPointCS0, wheelDirection_CS0, wheelAxle_CS, suspension_RestLength, wheel_Radius,
                            m_tuning, isFrontWheel);
        connectionPointCS0 =
            btVector3(CUBE_HALF_EXTENTS - (0.3 * wheel_Width), connectionHeight, -2 * CUBE_HALF_EXTENTS + wheel_Radius);
        m_vehicle->addWheel(connectionPointCS0, wheelDirection_CS0, wheelAxle_CS, suspension_RestLength, wheel_Radius,
                            m_tuning, isFrontWheel);

        auto wheelNum = m_vehicle->getNumWheels();

        for (int i = 0; i < wheelNum; i++)
        {
            btWheelInfo &wheel = m_vehicle->getWheelInfo(i);
            wheel.m_suspensionStiffness = suspension_Stiffness;
            wheel.m_wheelsDampingRelaxation = suspension_Damping;
            wheel.m_wheelsDampingCompression = suspension_Compression;
            wheel.m_frictionSlip = wheel_Friction;
            wheel.m_rollInfluence = roll_Influence;
        }
    }

    // seems not needed
    resetForklift();

    // practica
    for (size_t i = 0; i < TOWER_NUM; i++)
        createTower(SignedUnitRand() * 100.f, SignedUnitRand() * 100.f);

    m_wagon_0 = createVagon(m_vehicle);
    m_wagon_1 = createVagon(m_wagon_0);

    // draw stuff
    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void Hinge2Vehicle::physicsDebugDraw(int debugFlags)
{
    if (m_dynamicsWorld && m_dynamicsWorld->getDebugDrawer())
    {
        m_dynamicsWorld->getDebugDrawer()->setDebugMode(debugFlags);
        m_dynamicsWorld->debugDrawWorld();
    }
}

void Hinge2Vehicle::updateCamera()
{
    if (m_useDefaultCamera)
        return;

    btTransform chassisWorldTrans;

    // look at the vehicle
    m_carChassis->getMotionState()->getWorldTransform(chassisWorldTrans);
    m_cameraTargetPosition = chassisWorldTrans.getOrigin();

    btScalar posx = m_cameraTargetPosition.getX();
    btScalar posy = m_cameraTargetPosition.getY();
    btScalar posz = m_cameraTargetPosition.getZ();

    m_guiHelper->getRenderInterface()->getActiveCamera()->setCameraTargetPosition(posx, posy, posz);
}
// to be implemented by the demo
void Hinge2Vehicle::renderScene()
{
    updateCamera();

    auto *renderer = m_guiHelper->getRenderInterface();
    auto wheelCounter = 0;

    // thsi code is just to draw the fake wheels
    auto wheelNum = m_vehicle->getNumWheels();

    for (int i = 0; i < wheelNum; i++)
    {
        // synchronize the wheels with the (interpolated) chassis worldtransform
        m_vehicle->updateWheelTransform(i, true);

        if (!renderer)
            break;

        btTransform tr = m_vehicle->getWheelInfo(i).m_worldTransform;
        btVector3 pos = tr.getOrigin();
        btQuaternion orn = tr.getRotation();
        renderer->writeSingleInstanceTransformToCPU(pos, orn, m_wheelInstances[wheelCounter++]);
    }

    wheelNum = m_wagon_0->getNumWheels();

    for (int i = 0; i < wheelNum; i++)
    {
        // synchronize the wheels with the (interpolated) chassis worldtransform
        m_wagon_0->updateWheelTransform(i, true);

        if (!renderer)
            break;

        btTransform tr = m_wagon_0->getWheelInfo(i).m_worldTransform;
        btVector3 pos = tr.getOrigin();
        btQuaternion orn = tr.getRotation();
        renderer->writeSingleInstanceTransformToCPU(pos, orn, m_wheelInstances[wheelCounter++]);
    }

    wheelNum = m_wagon_1->getNumWheels();

    for (int i = 0; i < wheelNum; i++)
    {
        // synchronize the wheels with the (interpolated) chassis worldtransform
        m_wagon_1->updateWheelTransform(i, true);

        if (!renderer)
            break;

        btTransform tr = m_wagon_1->getWheelInfo(i).m_worldTransform;
        btVector3 pos = tr.getOrigin();
        btQuaternion orn = tr.getRotation();
        renderer->writeSingleInstanceTransformToCPU(pos, orn, m_wheelInstances[wheelCounter++]);
    }

    // needed because wheels transform is updated jsut before rendering them
    m_guiHelper->syncPhysicsToGraphics(m_dynamicsWorld);

    // now render everything
    m_guiHelper->render(m_dynamicsWorld);

#if 0
	ATTRIBUTE_ALIGNED16(btScalar)
	m[16];
	int i;

	btVector3 wheelColor(1, 0, 0);


	btVector3 worldBoundsMin, worldBoundsMax;
	getDynamicsWorld()->getBroadphase()->getBroadphaseAabb(worldBoundsMin, worldBoundsMax);

	for (i = 0; i < m_vehicle->getNumWheels(); i++)
	{
		//synchronize the wheels with the (interpolated) chassis worldtransform
		m_vehicle->updateWheelTransform(i, true);
		//draw wheels (cylinders)
		m_vehicle->getWheelInfo(i).m_worldTransform.getOpenGLMatrix(m);
		//		m_shapeDrawer->drawOpenGL(m,m_wheelShape,wheelColor,getDebugMode(),worldBoundsMin,worldBoundsMax);
	}


	int lineWidth=400;
	int xStart = m_glutScreenWidth - lineWidth;
	int yStart = 20;

	if((getDebugMode() & btIDebugDraw::DBG_NoHelpText)==0)
	{
		setOrthographicProjection();
		glDisable(GL_LIGHTING);
		glColor3f(0, 0, 0);
		char buf[124];
		
		sprintf(buf,"SHIFT+Cursor Left/Right - rotate lift");
		GLDebugDrawString(xStart,20,buf);
		yStart+=20;
		sprintf(buf,"SHIFT+Cursor UP/Down - fork up/down");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);

		if (m_useDefaultCamera)
		{
			sprintf(buf,"F5 - camera mode (free)");
		} else
		{
			sprintf(buf,"F5 - camera mode (follow)");
		}
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);

		yStart+=20;
		if (m_dynamicsWorld->getConstraintSolver()->getSolverType()==BT_MLCP_SOLVER)
		{
			sprintf(buf,"F6 - solver (direct MLCP)");
		} else
		{
			sprintf(buf,"F6 - solver (sequential impulse)");
		}
		GLDebugDrawString(xStart,yStart,buf);
		btDiscreteDynamicsWorld* world = (btDiscreteDynamicsWorld*) m_dynamicsWorld;
		if (world->getLatencyMotionStateInterpolation())
		{
			sprintf(buf,"F7 - motionstate interpolation (on)");
		} else
		{
			sprintf(buf,"F7 - motionstate interpolation (off)");
		}
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);

		sprintf(buf,"Click window for keyboard focus");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);


		resetPerspectiveProjection();
		glEnable(GL_LIGHTING);
	}
#endif
}

void Hinge2Vehicle::stepSimulation(float deltaTime)
{
    // glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    {
        int wheelIndex = 2;
        m_vehicle->applyEngineForce(g_EngineForce, wheelIndex);
        m_vehicle->setBrake(g_BreakingForce, wheelIndex);
        wheelIndex = 3;
        m_vehicle->applyEngineForce(g_EngineForce, wheelIndex);
        m_vehicle->setBrake(g_BreakingForce, wheelIndex);

        g_VehicleSteering += steering_Increment;
        if (g_VehicleSteering > steering_Clamp)
            g_VehicleSteering = steering_Clamp;
        if (g_VehicleSteering < -steering_Clamp)
            g_VehicleSteering = -steering_Clamp;

        wheelIndex = 0;
        m_vehicle->setSteeringValue(g_VehicleSteering, wheelIndex);
        wheelIndex = 1;
        m_vehicle->setSteeringValue(g_VehicleSteering, wheelIndex);
    }

    float dt = deltaTime;

    if (m_dynamicsWorld)
    {
        // during idle mode, just run 1 simulation step maximum
        int maxSimSubSteps = 2;

        int numSimSteps;
        numSimSteps = m_dynamicsWorld->stepSimulation(dt, maxSimSubSteps);

        if (m_dynamicsWorld->getConstraintSolver()->getSolverType() == BT_MLCP_SOLVER)
        {
            btMLCPSolver *sol = (btMLCPSolver *)m_dynamicsWorld->getConstraintSolver();
            int numFallbacks = sol->getNumFallbacks();
            if (numFallbacks)
            {
                static int totalFailures = 0;
                totalFailures += numFallbacks;
                printf("MLCP solver failed %d times, falling back to btSequentialImpulseSolver (SI)\n", totalFailures);
            }
            sol->setNumFallbacks(0);
        }

// #define VERBOSE_FEEDBACK
#ifdef VERBOSE_FEEDBACK
        if (!numSimSteps)
            printf("Interpolated transforms\n");
        else
        {
            if (numSimSteps > maxSimSubSteps)
            {
                // detect dropping frames
                printf("Dropped (%i) simulation steps out of %i\n", numSimSteps - maxSimSubSteps, numSimSteps);
            }
            else
            {
                printf("Simulated (%i) steps\n", numSimSteps);
            }
        }
#endif // VERBOSE_FEEDBACK
    }
}

void Hinge2Vehicle::displayCallback(void)
{
    //	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // renderme();

    // optional but useful: debug drawing
    if (m_dynamicsWorld)
        m_dynamicsWorld->debugDrawWorld();

    //	glFlush();
    //	glutSwapBuffers();
}

void Hinge2Vehicle::clientResetScene()
{
    exitPhysics();
    initPhysics();
}

void Hinge2Vehicle::resetForklift()
{
    g_VehicleSteering = 0.f;
    g_BreakingForce = default_BreakingForce;
    g_EngineForce = 0.f;

    m_carChassis->setCenterOfMassTransform(btTransform::getIdentity());
    m_carChassis->setLinearVelocity(btVector3(0, 0, 0));
    m_carChassis->setAngularVelocity(btVector3(0, 0, 0));
    m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(
        m_carChassis->getBroadphaseHandle(), getDynamicsWorld()->getDispatcher());
    if (m_vehicle)
    {
        m_vehicle->resetSuspension();
        for (int i = 0; i < m_vehicle->getNumWheels(); i++)
        {
            // synchronize the wheels with the (interpolated) chassis worldtransform
            m_vehicle->updateWheelTransform(i, true);
        }
    }
}

bool Hinge2Vehicle::keyboardCallback(int key, int state)
{
    bool handled = false;
    bool isShiftPressed = m_guiHelper->getAppInterface()->m_window->isModifierKeyPressed(B3G_SHIFT);

    if (state)
    {
        if (isShiftPressed)
        {
            switch (key)
            {
            case B3G_LEFT_ARROW: {
                handled = true;

                break;
            }
            case B3G_RIGHT_ARROW: {
                handled = true;
                break;
            }
            case B3G_UP_ARROW: {
                handled = true;
                break;
            }
            case B3G_DOWN_ARROW: {
                handled = true;
                break;
            }
            }
        }
        else
        {
            switch (key)
            {
            case B3G_LEFT_ARROW: {
                handled = true;
                steering_Increment = maxsteering_Increment;

                break;
            }
            case B3G_RIGHT_ARROW: {
                handled = true;
                steering_Increment = -maxsteering_Increment;
                break;
            }
            case B3G_UP_ARROW: {
                handled = true;
                g_EngineForce = max_EngineForce;
                g_BreakingForce = 0.f;
                break;
            }
            case B3G_DOWN_ARROW: {
                handled = true;
                g_EngineForce = -max_EngineForce;
                g_BreakingForce = 0.f;
                break;
            }

            case B3G_F7: {
                handled = true;
                btDiscreteDynamicsWorld *world = (btDiscreteDynamicsWorld *)m_dynamicsWorld;
                world->setLatencyMotionStateInterpolation(!world->getLatencyMotionStateInterpolation());
                printf("world latencyMotionStateInterpolation = %d\n", world->getLatencyMotionStateInterpolation());
                break;
            }
            case B3G_F6: {
                handled = true;
                // switch solver (needs demo restart)
                use_MCLPSolver = !use_MCLPSolver;
                printf("switching to useMLCPSolver = %d\n", use_MCLPSolver);

                delete m_constraintSolver;
                if (use_MCLPSolver)
                {
                    btDantzigSolver *mlcp = new btDantzigSolver();
                    // btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
                    btMLCPSolver *sol = new btMLCPSolver(mlcp);
                    m_constraintSolver = sol;
                }
                else
                {
                    m_constraintSolver = new btSequentialImpulseConstraintSolver();
                }

                m_dynamicsWorld->setConstraintSolver(m_constraintSolver);

                // exitPhysics();
                // initPhysics();
                break;
            }

            case B3G_F5:
                handled = true;
                m_useDefaultCamera = !m_useDefaultCamera;
                break;
            default:
                break;
            }
        }
    }
    else
    {
        switch (key)
        {
        case B3G_UP_ARROW: {
            g_EngineForce = 0.f;
            g_BreakingForce = default_BreakingForce;
            handled = true;
            break;
        }
        case B3G_DOWN_ARROW: {
            g_EngineForce = 0.f;
            g_BreakingForce = default_BreakingForce;
            handled = true;
            break;
        }
        case B3G_LEFT_ARROW:
        case B3G_RIGHT_ARROW: {
            steering_Increment = 0.0f;
            handled = true;
            break;
        }
        default:

            break;
        }
    }
    return handled;
}

void Hinge2Vehicle::specialKeyboardUp(int key, int x, int y)
{
#if 0

#endif
}

void Hinge2Vehicle::specialKeyboard(int key, int x, int y)
{
#if 0
	if (key==GLUT_KEY_END)
		return;

	//	printf("key = %i x=%i y=%i\n",key,x,y);

	int state;
	state=glutGetModifiers();
	if (state & GLUT_ACTIVE_SHIFT) 
	{
		switch (key) 
			{
			case GLUT_KEY_LEFT : 
				{
				
					m_liftHinge->setLimit(-M_PI/16.0f, M_PI/8.0f);
					m_liftHinge->enableAngularMotor(true, -0.1, max_MotorImpulse);
					break;
				}
			case GLUT_KEY_RIGHT : 
				{
					
					m_liftHinge->setLimit(-M_PI/16.0f, M_PI/8.0f);
					m_liftHinge->enableAngularMotor(true, 0.1, max_MotorImpulse);
					break;
				}
			case GLUT_KEY_UP :
				{
					m_forkSlider->setLowerLinLimit(0.1f);
					m_forkSlider->setUpperLinLimit(3.9f);
					m_forkSlider->setPoweredLinMotor(true);
					m_forkSlider->setMaxLinMotorForce(max_MotorImpulse);
					m_forkSlider->setTargetLinMotorVelocity(1.0);
					break;
				}
			case GLUT_KEY_DOWN :
				{
					m_forkSlider->setLowerLinLimit(0.1f);
					m_forkSlider->setUpperLinLimit(3.9f);
					m_forkSlider->setPoweredLinMotor(true);
					m_forkSlider->setMaxLinMotorForce(max_MotorImpulse);
					m_forkSlider->setTargetLinMotorVelocity(-1.0);
					break;
				}

			default:
				DemoApplication::specialKeyboard(key,x,y);
				break;
			}

	} else
	{
			switch (key) 
			{
			case GLUT_KEY_LEFT : 
				{
					g_VehicleSteering += steering_Increment;
					if (	g_VehicleSteering > steering_Clamp)
						g_VehicleSteering = steering_Clamp;

					break;
				}
			case GLUT_KEY_RIGHT : 
				{
					g_VehicleSteering -= steering_Increment;
					if (	g_VehicleSteering < -steering_Clamp)
						g_VehicleSteering = -steering_Clamp;

					break;
				}
			case GLUT_KEY_UP :
				{
					g_EngineForce = max_EngineForce;
					g_BreakingForce = 0.f;
					break;
				}
			case GLUT_KEY_DOWN :
				{
					g_EngineForce = -max_EngineForce;
					g_BreakingForce = 0.f;
					break;
				}

			case GLUT_KEY_F7:
				{
					btDiscreteDynamicsWorld* world = (btDiscreteDynamicsWorld*)m_dynamicsWorld;
					world->setLatencyMotionStateInterpolation(!world->getLatencyMotionStateInterpolation());
					printf("world latencyMotionStateInterpolation = %d\n", world->getLatencyMotionStateInterpolation());
					break;
				}
			case GLUT_KEY_F6:
				{
					//switch solver (needs demo restart)
					use_MCLPSolver = !use_MCLPSolver;
					printf("switching to useMLCPSolver = %d\n", use_MCLPSolver);

					delete m_constraintSolver;
					if (use_MCLPSolver)
					{
						btDantzigSolver* mlcp = new btDantzigSolver();
						//btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
						btMLCPSolver* sol = new btMLCPSolver(mlcp);
						m_constraintSolver = sol;
					} else
					{
						m_constraintSolver = new btSequentialImpulseConstraintSolver();
					}

					m_dynamicsWorld->setConstraintSolver(m_constraintSolver);


					//exitPhysics();
					//initPhysics();
					break;
				}

			case GLUT_KEY_F5:
				m_useDefaultCamera = !m_useDefaultCamera;
				break;
			default:
				DemoApplication::specialKeyboard(key,x,y);
				break;
			}

	}
	//	glutPostRedisplay();

#endif
}

btRigidBody *Hinge2Vehicle::localCreateRigidBody(btScalar mass, const btTransform &startTransform,
                                                 btCollisionShape *shape)
{
    btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

    // rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
        shape->calculateLocalInertia(mass, localInertia);

        // using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active'
        // objects

#define USE_MOTIONSTATE 1
#ifdef USE_MOTIONSTATE
    btDefaultMotionState *myMotionState = new btDefaultMotionState(startTransform);

    btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);

    btRigidBody *body = new btRigidBody(cInfo);
    // body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

#else
    btRigidBody *body = new btRigidBody(mass, 0, shape, localInertia);
    body->setWorldTransform(startTransform);
#endif //

    m_dynamicsWorld->addRigidBody(body);
    return body;
}

CommonExampleInterface *Hinge2VehicleCreateFunc(struct CommonExampleOptions &options)
{
    return new Hinge2Vehicle(options.m_guiHelper);
}
