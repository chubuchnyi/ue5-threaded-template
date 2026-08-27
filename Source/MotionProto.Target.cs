using UnrealBuildTool;

public class MotionProtoTarget : TargetRules
{
	public MotionProtoTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Game;
		DefaultBuildSettings = BuildSettingsVersion.V6;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
		ExtraModuleNames.Add("MotionProto");
	}
}
