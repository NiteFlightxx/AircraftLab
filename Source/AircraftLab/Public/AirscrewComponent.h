#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "DroneTypes.h"

#include "AirscrewComponent.generated.h"

class UPrimitiveComponent;

UCLASS(ClassGroup = (AircraftLab), meta = (BlueprintSpawnableComponent))
class AIRCRAFTLAB_API UAirscrewComponent : public USceneComponent
{
	GENERATED_BODY()

public:
	UAirscrewComponent();

	virtual void OnRegister() override;
	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	UFUNCTION(BlueprintCallable, Category = "Drone|Airscrew")
	void SetNormalizedCommand(float InNormalizedCommand);

	UFUNCTION(BlueprintCallable, Category = "Drone|Airscrew")
	void SetRotorEnabled(bool bNewEnabled);

	/** 设置旋翼效能（0~1），0=完全失效电机停转，1=正常，中间值模拟不同程度损坏 */
	UFUNCTION(BlueprintCallable, Category = "Drone|Airscrew")
	void SetRotorEffectiveness(float Effectiveness);

	/** 快捷方式：设置旋翼完全失效（效能=0） */
	UFUNCTION(BlueprintCallable, Category = "Drone|Airscrew")
	void FailRotor();

	/** 快捷方式：恢复旋翼正常效能（效能=1） */
	UFUNCTION(BlueprintCallable, Category = "Drone|Airscrew")
	void RestoreRotor();

	UFUNCTION(BlueprintCallable, Category = "Drone|Airscrew")
	void SetForceApplicationEnabled(bool bNewEnabled);

	UFUNCTION(BlueprintCallable, Category = "Drone|Airscrew")
	void SetDebugDrawEnabled(bool bNewEnabled);

	UFUNCTION(BlueprintPure, Category = "Drone|Airscrew")
	float GetNormalizedCommand() const { return TargetNormalizedCommand; }

	UFUNCTION(BlueprintPure, Category = "Drone|Airscrew")
	float GetCurrentCommand() const { return CurrentNormalizedCommand; }

	UFUNCTION(BlueprintPure, Category = "Drone|Airscrew")
	float GetCurrentRpm() const { return CurrentRpm; }

	UFUNCTION(BlueprintPure, Category = "Drone|Airscrew")
	float GetCurrentThrustForce() const { return CurrentThrustForce; }

	UFUNCTION(BlueprintPure, Category = "Drone|Airscrew")
	FVector GetCurrentThrustVectorWorld() const { return CurrentThrustVectorWorld; }

	UFUNCTION(BlueprintPure, Category = "Drone|Airscrew")
	FVector GetCurrentApplicationPointWorld() const { return CurrentApplicationPointWorld; }

	UFUNCTION(BlueprintPure, Category = "Drone|Airscrew")
	float GetCurrentReactionTorqueMagnitude() const { return CurrentReactionTorqueMagnitude; }

	UFUNCTION(BlueprintPure, Category = "Drone|Airscrew")
	FVector GetCurrentReactionTorqueVectorWorld() const { return CurrentReactionTorqueVectorWorld; }

	const FDroneRotorDefinition& GetRotorDefinition() const { return RotorDefinition; }
	bool IsRotorEnabled() const { return RotorDefinition.IsEnabled() && RotorEffectiveness > 0.0f; }
	float GetRotorEffectiveness() const { return RotorEffectiveness; }

public:
	void SyncDefinitionFromComponentTransform();
	void UpdateRotorState(float DeltaTime);
	void ApplyThrustForce();
	void DrawDebugVisualization() const;

	UPrimitiveComponent* ResolveTargetPrimitive() const;
	FVector GetThrustDirectionWorld() const;
	float GetEffectiveTargetCommand() const;
	float ComputeTargetRpm(float EffectiveCommand) const;

protected:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Airscrew")
	FDroneRotorDefinition RotorDefinition;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Airscrew")
	bool bApplyForce = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Airscrew", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float TargetNormalizedCommand = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Airscrew", meta = (ClampMin = "0.0"))
	float CommandScale = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Debug")
	bool bDrawDebug = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Debug")
	bool bDrawDebugText = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Debug", meta = (ClampMin = "0.0"))
	float DebugForceScale = 0.1f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Debug", meta = (ClampMin = "0.0"))
	float DebugAxisLength = 30.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Debug", meta = (ClampMin = "0.0"))
	float DebugTextOffset = 18.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Debug")
	FLinearColor DebugEnabledColor = FLinearColor(0.0f, 1.0f, 0.2f, 1.0f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Debug")
	FLinearColor DebugDisabledColor = FLinearColor(0.35f, 0.35f, 0.35f, 1.0f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Debug")
	FLinearColor DebugFailedColor = FLinearColor(1.0f, 0.1f, 0.1f, 1.0f);

	/** 旋翼效能（0~1）：1=正常，0=完全失效，中间值表示部分损坏。
	 *  效能>0时电机仍响应指令但推力按比例缩放；效能=0时电机停转RPM自然衰减 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Airscrew", meta = (AllowPrivateAccess = "true", ClampMin = "0.0", ClampMax = "1.0"))
	float RotorEffectiveness = 1.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Airscrew", meta = (AllowPrivateAccess = "true"))
	float CurrentNormalizedCommand = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Airscrew", meta = (AllowPrivateAccess = "true"))
	float CurrentRpm = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Airscrew", meta = (AllowPrivateAccess = "true"))
	float CurrentThrustForce = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Airscrew", meta = (AllowPrivateAccess = "true"))
	FVector CurrentThrustVectorWorld = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Airscrew", meta = (AllowPrivateAccess = "true"))
	FVector CurrentApplicationPointWorld = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Airscrew", meta = (AllowPrivateAccess = "true"))
	float CurrentReactionTorqueMagnitude = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Airscrew", meta = (AllowPrivateAccess = "true"))
	FVector CurrentReactionTorqueVectorWorld = FVector::ZeroVector;
};
