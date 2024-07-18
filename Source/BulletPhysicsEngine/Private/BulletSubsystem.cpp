#include "BulletSubsystem.h"
#include "../../../../../Source/CarPluginProject/Public/XECSClasses/XECSBullet.h"

void UBulletSubsystem::Initialize(FSubsystemCollectionBase& Collection) {
	BtCollisionConfig = new btDefaultCollisionConfiguration();

	BtCollisionDispatcher = new btCollisionDispatcher(BtCollisionConfig);
	BtBroadphase = new btDbvtBroadphase();
	mt = new btSequentialImpulseConstraintSolver;
	mt->setRandSeed(1234);
	BtConstraintSolver = mt;


	BtWorld = new btDiscreteDynamicsWorld(BtCollisionDispatcher, BtBroadphase, BtConstraintSolver, BtCollisionConfig);
	BtWorld->setGravity(btVector3(0, 0, -9.8));
	Super::Initialize(Collection);
}
void UBulletSubsystem::Deinitialize() {
	Super::Deinitialize();
	ResetSim();
	/*
	delete BtCollisionConfig;
	delete BtCollisionDispatcher;
	delete BtBroadphase;
	delete mt;
	delete BtWorld;*/
}
void UBulletSubsystem::Tick(float DeltaTime) {

}

void UBulletSubsystem::TriggerOnPhysicsTick(float DeltaTime)
{
	// Trigger the delegate, passing the new value
	OnPhysicsTick.Broadcast(DeltaTime);
}

FVector UBulletSubsystem::GetVelocityAtWorldPoint(int ID, FVector WorldLocation) {
	const auto& body = BtRigidBodies[ID];
	const auto CentreOfMass = body->getCenterOfMassPosition();
	
	return BulletHelpers::ToUEDir(body->getLinearVelocity() + btCross(body->getAngularVelocity(), BulletHelpers::ToBtPos(WorldLocation) - CentreOfMass));
}

void UBulletSubsystem::SetupStaticGeometryPhysics(TArray<AActor*> Actors, float Friction, float Restitution)
{
	for (AActor* Actor : Actors)
	{
		ExtractPhysicsGeometry(Actor,
			[Actor, this, Friction, Restitution](btCollisionShape* Shape, const FTransform& RelTransform)
			{
				// Every sub-collider in the actor is passed to this callback function
				// We're baking this in world space, so apply actor transform to relative
				const FTransform FinalXform = RelTransform * Actor->GetActorTransform();
				AddStaticCollision(Shape, FinalXform, Friction, Restitution, Actor);
			});
	}
}


void UBulletSubsystem::AddStaticBody(AActor* Body, float Friction, float Restitution, int& ID)
{
	ExtractPhysicsGeometry(Body, [Body, this, Friction, Restitution](btCollisionShape* Shape, const FTransform& RelTransform)
		{
			// Every sub-collider in the actor is passed to this callback function
			// We're baking this in world space, so apply actor transform to relative
			const FTransform FinalXform = RelTransform * Body->GetActorTransform();
			AddStaticCollision(Shape, FinalXform, Friction, Restitution, Body);
		});

	ID = BtWorld->getNumCollisionObjects() - 1;
}


void UBulletSubsystem::AddProcBody(AActor* Body, float Friction, TArray<FVector> a, TArray<FVector> b, TArray<FVector> c, TArray<FVector> d, float Restitution, int& ID)
{

	btCollisionShape* Shape = GetTriangleMeshShape(a, b, c, d);
	const FTransform FinalXform = Body->GetActorTransform();
	procbody = AddStaticCollision(Shape, FinalXform, Friction, Restitution, Body);


	ID = BtWorld->getNumCollisionObjects() - 1;
}


void UBulletSubsystem::UpdateProcBody(AActor* Body, float Friction, TArray<FVector> a, TArray<FVector> b, TArray<FVector> c, TArray<FVector> d, float Restitution, int& ID, int PrevID)
{
	BtWorld->removeCollisionObject(procbody);
	procbody = nullptr;

	btCollisionShape* Shape = GetTriangleMeshShape(a, b, c, d);
	const FTransform FinalXform = Body->GetActorTransform();
	procbody = AddStaticCollision(Shape, FinalXform, Friction, Restitution, Body);



	ID = BtWorld->getNumCollisionObjects() - 1;
}

void UBulletSubsystem::UpdatePlayertransform(AActor* player, int ID)
{
	BtWorld->getCollisionObjectArray()[ID]->setWorldTransform(BulletHelpers::ToBt(player->GetActorTransform()));
}


void UBulletSubsystem::AddImpulse(int ID, FVector Impulse, FVector Location)
{
	BtRigidBodies[ID]->applyImpulse(BulletHelpers::ToBtDir(Impulse, true), BulletHelpers::ToBtPos(Location));
}

void UBulletSubsystem::AddImpulse(const FXECSBulletObject& ID, FVector Impulse, FVector Location)
{
	ID.body->applyImpulse(BulletHelpers::ToBtDir(Impulse, true), BulletHelpers::ToBtPos(Location));
}



void UBulletSubsystem::ExtractPhysicsGeometry(AActor* Actor, PhysicsGeometryCallback CB)
{
	TInlineComponentArray<UActorComponent*, 20> Components;
	// Used to easily get a component's transform relative to actor, not parent component
	const FTransform InvActorTransform = Actor->GetActorTransform().Inverse();

	// Get From Skeletal Mesh
	Actor->GetComponents(USkeletalMeshComponent::StaticClass(), Components);
	for (auto&& Comp : Components)
	{
		ExtractPhysicsGeometry(Cast<USkeletalMeshComponent>(Comp), InvActorTransform, CB);
	}

	// Collisions from meshes
	Actor->GetComponents(UStaticMeshComponent::StaticClass(), Components);
	for (auto&& Comp : Components)
	{
		ExtractPhysicsGeometry(Cast<UStaticMeshComponent>(Comp), InvActorTransform, CB);
	}

	// Collisions from separate collision components
	Actor->GetComponents(UShapeComponent::StaticClass(), Components);
	for (auto&& Comp : Components)
	{
		ExtractPhysicsGeometry(Cast<UShapeComponent>(Comp), InvActorTransform, CB);
	}
}


btCollisionObject* UBulletSubsystem::AddStaticCollision(btCollisionShape* Shape, const FTransform& Transform, float Friction,
	float Restitution, AActor* Actor)
{
	btTransform Xform = BulletHelpers::ToBt(Transform);
	btCollisionObject* Obj = new btCollisionObject();
	Obj->setCollisionShape(Shape);
	Obj->setWorldTransform(Xform);
	Obj->setFriction(Friction);
	Obj->setRestitution(Restitution);
	Obj->setUserPointer(Actor);
	Obj->setActivationState(DISABLE_DEACTIVATION);
	BtWorld->addCollisionObject(Obj);
	UE_LOG(LogTemp, Warning, TEXT("Static geom added"));
	BtStaticObjects.Add(Obj);

	return Obj;
}


void UBulletSubsystem::ExtractPhysicsGeometry(UStaticMeshComponent* SMC, const FTransform& InvActorXform, PhysicsGeometryCallback CB)
{
	UStaticMesh* Mesh = SMC->GetStaticMesh();
	if (!Mesh)
		return;

	// We want the complete transform from actor to this component, not just relative to parent
	FTransform CompFullRelXForm = SMC->GetComponentTransform() * InvActorXform;
	ExtractPhysicsGeometry(CompFullRelXForm, Mesh->GetBodySetup(), CB);

	// Not supporting complex collision shapes right now
	// If we did, note that Mesh->ComplexCollisionMesh is WITH_EDITORONLY_DATA so not available at runtime
	// See StaticMeshRender.cpp, FStaticMeshSceneProxy::GetDynamicMeshElements
	// Line 1417+, bDrawComplexCollision
	// Looks like we have to access LODForCollision, RenderData->LODResources
	// So they use a mesh LOD for collision for complex shapes, never drawn usually?

}

void UBulletSubsystem::ExtractPhysicsGeometry(USkeletalMeshComponent* SMC, const FTransform& InvActorXform, PhysicsGeometryCallback CB)
{
	USkeletalMesh* Mesh = SMC->GetSkeletalMeshAsset();
	if (!Mesh)
		return;

	// We want the complete transform from actor to this component, not just relative to parent
	FTransform CompFullRelXForm = SMC->GetComponentTransform() * InvActorXform;
	ExtractPhysicsGeometry(CompFullRelXForm, Mesh->GetBodySetup(), CB);

	// Not supporting complex collision shapes right now
	// If we did, note that Mesh->ComplexCollisionMesh is WITH_EDITORONLY_DATA so not available at runtime
	// See StaticMeshRender.cpp, FStaticMeshSceneProxy::GetDynamicMeshElements
	// Line 1417+, bDrawComplexCollision
	// Looks like we have to access LODForCollision, RenderData->LODResources
	// So they use a mesh LOD for collision for complex shapes, never drawn usually?

}

bool UBulletSubsystem::CastRay(
	const FVector& origin,
	const FVector& direction,
	const float length)
{
	btVector3 from(origin.X, origin.Y, origin.Z);
	auto toFVec = origin + direction * length;
	btVector3 to(toFVec.X, toFVec.Y, toFVec.Z);
	btCollisionWorld::AllHitsRayResultCallback allResults(from, to);
	BtWorld->rayTest(from, to, allResults);
	return allResults.hasHit();
}


void UBulletSubsystem::ExtractPhysicsGeometry(UShapeComponent* Sc, const FTransform& InvActorXform, PhysicsGeometryCallback CB)
{
	// We want the complete transform from actor to this component, not just relative to parent
	FTransform CompFullRelXForm = Sc->GetComponentTransform() * InvActorXform;
	ExtractPhysicsGeometry(CompFullRelXForm, Sc->ShapeBodySetup, CB);
}


void UBulletSubsystem::ExtractPhysicsGeometry(const FTransform& XformSoFar, UBodySetup* BodySetup, PhysicsGeometryCallback CB)
{
	FVector Scale = XformSoFar.GetScale3D();
	btCollisionShape* Shape = nullptr;

	// Iterate over the simple collision shapes
	for (auto&& Box : BodySetup->AggGeom.BoxElems)
	{
		// We'll re-use based on just the LxWxH, including actor scale
		// Rotation and centre will be baked in world space
		FVector Dimensions = FVector(Box.X, Box.Y, Box.Z) * Scale;
		Shape = GetBoxCollisionShape(Dimensions);
		FTransform ShapeXform(Box.Rotation, Box.Center);
		// Shape transform adds to any relative transform already here
		FTransform XForm = ShapeXform * XformSoFar;
		CB(Shape, XForm);
	}
	for (auto&& Sphere : BodySetup->AggGeom.SphereElems)
	{
		// Only support uniform scale so use X
		Shape = GetSphereCollisionShape(Sphere.Radius * Scale.X);
		FTransform ShapeXform(FRotator::ZeroRotator, Sphere.Center);
		// Shape transform adds to any relative transform already here
		FTransform XForm = ShapeXform * XformSoFar;
		CB(Shape, XForm);
	}
	// Sphyl == Capsule (??)
	for (auto&& Capsule : BodySetup->AggGeom.SphylElems)
	{
		// X scales radius, Z scales height
		Shape = GetCapsuleCollisionShape(Capsule.Radius * Scale.X, Capsule.Length * Scale.Z);
		// Capsules are in Z in UE, in Y in Bullet, so roll -90
		FRotator Rot(0, 0, -90);
		// Also apply any local rotation
		Rot += Capsule.Rotation;
		FTransform ShapeXform(Rot, Capsule.Center);
		// Shape transform adds to any relative transform already here
		FTransform XForm = ShapeXform * XformSoFar;
		CB(Shape, XForm);
	}
	for (int i = 0; i < BodySetup->AggGeom.ConvexElems.Num(); ++i)
	{
		Shape = GetConvexHullCollisionShape(BodySetup, i, Scale);
		CB(Shape, XformSoFar);
	}

}

btCollisionShape* UBulletSubsystem::GetBoxCollisionShape(const FVector& Dimensions)
{
	// Simple brute force lookup for now, probably doesn't need anything more clever
	btVector3 HalfSize = BulletHelpers::ToBtSize(Dimensions * 0.5);
	for (auto&& S : BtBoxCollisionShapes)
	{
		btVector3 Sz = S->getHalfExtentsWithMargin();
		if (FMath::IsNearlyEqual(Sz.x(), HalfSize.x()) &&
			FMath::IsNearlyEqual(Sz.y(), HalfSize.y()) &&
			FMath::IsNearlyEqual(Sz.z(), HalfSize.z()))
		{
			return S;
		}
	}

	// Not found, create
	auto S = new btBoxShape(HalfSize);
	// Get rid of margins, just cause issues for me
	S->setMargin(0);
	BtBoxCollisionShapes.Add(S);

	return S;

}

btCollisionShape* UBulletSubsystem::GetSphereCollisionShape(float Radius)
{
	// Simple brute force lookup for now, probably doesn't need anything more clever
	btScalar Rad = BulletHelpers::ToBtSize(Radius);
	for (auto&& S : BtSphereCollisionShapes)
	{
		// Bullet subtracts a margin from its internal shape, so add back to compare
		if (FMath::IsNearlyEqual(S->getRadius(), Rad))
		{
			return S;
		}
	}

	// Not found, create
	auto S = new btSphereShape(Rad);
	// Get rid of margins, just cause issues for me
	S->setMargin(0);
	BtSphereCollisionShapes.Add(S);

	return S;

}

btCollisionShape* UBulletSubsystem::GetCapsuleCollisionShape(float Radius, float Height)
{
	// Simple brute force lookup for now, probably doesn't need anything more clever
	btScalar R = BulletHelpers::ToBtSize(Radius);
	btScalar H = BulletHelpers::ToBtSize(Height);
	btScalar HalfH = H * 0.5f;

	for (auto&& S : BtCapsuleCollisionShapes)
	{
		// Bullet subtracts a margin from its internal shape, so add back to compare
		if (FMath::IsNearlyEqual(S->getRadius(), R) &&
			FMath::IsNearlyEqual(S->getHalfHeight(), HalfH))
		{
			return S;
		}
	}

	// Not found, create
	auto S = new btCapsuleShape(R, H);
	BtCapsuleCollisionShapes.Add(S);

	return S;

}

btCollisionShape* UBulletSubsystem::GetTriangleMeshShape(TArray<FVector> a, TArray<FVector> b, TArray<FVector> c, TArray<FVector> d)
{
	btTriangleMesh* triangleMesh = new btTriangleMesh();

	for (int i = 0; i < a.Num(); i++)
	{
		triangleMesh->addTriangle(BulletHelpers::ToBtPos(a[i]), BulletHelpers::ToBtPos(b[i]), BulletHelpers::ToBtPos(c[i]));
		triangleMesh->addTriangle(BulletHelpers::ToBtPos(a[i]), BulletHelpers::ToBtPos(c[i]), BulletHelpers::ToBtPos(d[i]));

	}
	btBvhTriangleMeshShape* Trimesh = new btBvhTriangleMeshShape(triangleMesh, true);
	return Trimesh;
}

btCollisionShape* UBulletSubsystem::GetConvexHullCollisionShape(UBodySetup* BodySetup, int ConvexIndex, const FVector& Scale)
{
	for (auto&& S : BtConvexHullCollisionShapes)
	{
		if (S.BodySetup == BodySetup && S.HullIndex == ConvexIndex && S.Scale.Equals(Scale))
		{
			return S.Shape;
		}
	}

	const FKConvexElem& Elem = BodySetup->AggGeom.ConvexElems[ConvexIndex];
	auto C = new btConvexHullShape();
	for (auto&& P : Elem.VertexData)
	{
		C->addPoint(BulletHelpers::ToBtPos(P));
	}
	// Very important! Otherwise there's a gap between 
	C->setMargin(0);
	// Apparently this is good to call?
	C->initializePolyhedralFeatures();

	BtConvexHullCollisionShapes.Add({
		BodySetup,
		ConvexIndex,
		Scale,
		C
		});

	return C;
}


const UBulletSubsystem::CachedDynamicShapeData& UBulletSubsystem::GetCachedDynamicShapeData(AActor* Actor, float Mass)
{
	// We re-use compound shapes based on (leaf) BP class
	const FName ClassName = Actor->GetClass()->GetFName();



	// Because we want to support compound colliders, we need to extract all colliders first before
	// constructing the final body.
	TArray<btCollisionShape*, TInlineAllocator<20>> Shapes;
	TArray<FTransform, TInlineAllocator<20>> ShapeRelXforms;
	ExtractPhysicsGeometry(Actor,
		[&Shapes, &ShapeRelXforms](btCollisionShape* Shape, const FTransform& RelTransform)
		{
			Shapes.Add(Shape);
			ShapeRelXforms.Add(RelTransform);
		});


	CachedDynamicShapeData ShapeData;
	ShapeData.ClassName = ClassName;

	// Single shape with no transform is simplest
	if (ShapeRelXforms.Num() == 1 &&
		ShapeRelXforms[0].EqualsNoScale(FTransform::Identity))
	{
		ShapeData.Shape = Shapes[0];
		// just to make sure we don't think we have to clean it up; simple shapes are already stored
		ShapeData.bIsCompound = false;
	}
	else
	{
		// Compound or offset single shape; we will cache these by blueprint type
		btCompoundShape* CS = new btCompoundShape();
		for (int i = 0; i < Shapes.Num(); ++i)
		{
			// We don't use the actor origin when converting transform in this case since object space
			// Note that btCompoundShape doesn't free child shapes, which is fine since they're tracked separately
			CS->addChildShape(BulletHelpers::ToBt(ShapeRelXforms[i]), Shapes[i]);
		}

		ShapeData.Shape = CS;
		ShapeData.bIsCompound = true;
	}

	// Calculate Inertia
	ShapeData.Mass = Mass;
	ShapeData.Shape->calculateLocalInertia(Mass, ShapeData.Inertia);

	// Cache for future use
	CachedDynamicShapes.Add(ShapeData);

	return CachedDynamicShapes.Last();

}


FXECSBulletObject UBulletSubsystem::AddRigidBody(AActor* Body, float Friction, float Restitution, float mass)
{
	FXECSBulletObject object;
	object.bulletID = BtRigidBodies.Num();
	object.body =
		AddRigidBody(Body, GetCachedDynamicShapeData(Body, mass), Friction, Restitution);
	return object;
}

btRigidBody* UBulletSubsystem::AddRigidBody(AActor* Actor, const UBulletSubsystem::CachedDynamicShapeData& ShapeData, float Friction, float Restitution)
{
	return AddRigidBody(Actor, ShapeData.Shape, ShapeData.Inertia, ShapeData.Mass, Friction, Restitution);
}
btRigidBody* UBulletSubsystem::AddRigidBody(AActor* Actor, btCollisionShape* CollisionShape, btVector3 Inertia, float Mass, float Friction, float Restitution)
{
	auto MotionState = new BulletCustomMotionState(Actor);
	return AddRigidBodyWithCustomMotionState(MotionState, CollisionShape, Inertia, Mass, Friction, Restitution);
}
btRigidBody* UBulletSubsystem::AddRigidBodyWithCustomMotionState(BulletCustomMotionState* MotionState, btCollisionShape* CollisionShape, btVector3 Inertia, float Mass, float Friction, float Restitution)
{
	const btRigidBody::btRigidBodyConstructionInfo rbInfo(Mass * 10, MotionState, CollisionShape, Inertia * 10);
	btRigidBody* Body = new btRigidBody(rbInfo);
	Body->setActivationState(ACTIVE_TAG);
	Body->setDeactivationTime(0);

	BtWorld->addRigidBody(Body);
	BtRigidBodies.Add(Body);

	return Body;
}

FXECSBulletObject UBulletSubsystem::AddCylinderBody(AActor* Actor, const FVector& Inertia, const FTransform& startTransform, float wheelWidth, float wheelRadius, float Friction, float Restitution, float mass) {
	FXECSBulletObject object;
	object.bulletID = BtRigidBodies.Num();
	btCollisionShape* m_wheelShape = new btCylinderShapeZ(btVector3(BulletHelpers::ToBtSize(wheelRadius), BulletHelpers::ToBtSize(wheelRadius), BulletHelpers::ToBtSize(wheelWidth * 0.5)));
	btRigidBody* body = AddRigidBodyWithCustomMotionState(nullptr, m_wheelShape, BulletHelpers::ToBtMomentOfInertia(Inertia), mass, Friction, Restitution);
	body->setContactStiffnessAndDamping(3, 1);
	object.body = body;
	return object;
}

FXECSBulletObject UBulletSubsystem::AddCylinderBody(const FTransform& startTransform, float wheelWidth, float wheelRadius, float Friction, float Restitution, float mass) {
	btCollisionShape* m_wheelShape = new btCylinderShapeZ(btVector3(BulletHelpers::ToBtSize(wheelRadius), BulletHelpers::ToBtSize(wheelRadius), BulletHelpers::ToBtSize(wheelWidth * 0.5)));
	return createRigidBody(mass, BulletHelpers::ToBt(startTransform), m_wheelShape, Friction, Restitution);
}

FXECSBulletObject UBulletSubsystem::createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape, float Friction, float Restitution)
{
	FXECSBulletObject object;
	object.bulletID = BtRigidBodies.Num();
	btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		shape->calculateLocalInertia(mass, localInertia);

	btRigidBody::btRigidBodyConstructionInfo info(mass, 0, shape);
	btRigidBody* body = new btRigidBody(info);
	body->setWorldTransform(startTransform);
	body->setContactStiffnessAndDamping(3000000, 100);

	body->setUserIndex(-1);
	BtWorld->addRigidBody(body);
	object.body = body;
	BtRigidBodies.Add(body);
	return object;
}

void UBulletSubsystem::StepPhysics(float DeltaSeconds, int substeps)
{
	BtWorld->stepSimulation(DeltaSeconds, substeps, 1. / 60);
	TriggerOnPhysicsTick(DeltaSeconds);
}

void UBulletSubsystem::SetPhysicsState(int ID, FTransform transforms, FVector Velocity, FVector AngularVelocity, FVector& Force)
{

	if (BtRigidBodies[ID]) {
		BtRigidBodies[ID]->setWorldTransform(BulletHelpers::ToBt(transforms));
		BtRigidBodies[ID]->setLinearVelocity(BulletHelpers::ToBtPos(Velocity));
		BtRigidBodies[ID]->setAngularVelocity(BulletHelpers::ToBtPos(AngularVelocity));
		//BtRigidBodies[ID]->apply(BulletHelpers::ToBtPos(Velocity, GetActorLocation()));
	}


}

void UBulletSubsystem::GetPhysicsState(int ID, FTransform& transforms, FVector& Velocity, FVector& AngularVelocity, FVector& Force)
{
	if (BtRigidBodies[ID]) {
		transforms = BulletHelpers::ToUE(BtRigidBodies[ID]->getWorldTransform());
		Velocity = BulletHelpers::ToUEPos(BtRigidBodies[ID]->getLinearVelocity());
		AngularVelocity = BulletHelpers::ToUEPos(BtRigidBodies[ID]->getAngularVelocity());
		Force = BulletHelpers::ToUEPos(BtRigidBodies[ID]->getTotalForce());
		//AngularForce = BulletHelpers::ToUEPos(BtRigidBodies[ID]->getAngularVelocity(), FVector(0));
	}
}

FTransform UBulletSubsystem::GetPhysicsTransform(int ID)
{
	if (BtRigidBodies[ID]) {
		return BulletHelpers::ToUE(BtRigidBodies[ID]->getWorldTransform());
	}
	return FTransform();
}

void UBulletSubsystem::ResetSim()
{

	for (int i = 0; i < BtRigidBodies.Num(); i++)
	{
		BtWorld->removeRigidBody(BtRigidBodies[i]);
		BtRigidBodies[i]->setActivationState(ACTIVE_TAG);
		BtRigidBodies[i]->setDeactivationTime(0);
		BtRigidBodies[i]->clearForces();
	}

	BtWorld = new btDiscreteDynamicsWorld(BtCollisionDispatcher, BtBroadphase, BtConstraintSolver, BtCollisionConfig);
	BtWorld->setGravity(btVector3(0, 0, -9.8));
	BtBroadphase->resetPool(BtCollisionDispatcher);
	BtConstraintSolver->reset();
	for (int i = 0; i < BtRigidBodies.Num(); i++)
	{
		BtWorld->addRigidBody(BtRigidBodies[i]);
	}


}
