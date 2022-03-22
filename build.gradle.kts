import org.cadixdev.gradle.licenser.LicenseExtension

plugins {
    id("net.researchgate.release") version "2.8.1"
    id("org.cadixdev.licenser") version "0.5.1"
    `java-library`
    `maven-publish`
}

java {
    toolchain {
        languageVersion.set(JavaLanguageVersion.of(11))
    }
    withSourcesJar()
    withJavadocJar()
}

repositories {
    maven {
        name = "WPI Maven"
        url = uri("https://frcmaven.wpi.edu/artifactory/release")
    }
    maven {
        name = "CTRE Maven"
        url = uri("https://devsite.ctr-electronics.com/maven/release/")
    }
    maven {
        name = "Rev Maven"
        url = uri("https://maven.revrobotics.com/");
    }
}

dependencies {
    val wpiVersion = "2022.4.1"
    api("edu.wpi.first.wpilibj:wpilibj-java:$wpiVersion")
    api("edu.wpi.first.wpiutil:wpiutil-java:$wpiVersion")
    api("edu.wpi.first.ntcore:ntcore-java:$wpiVersion")
    api("edu.wpi.first.cscore:cscore-java:$wpiVersion")
    api("edu.wpi.first.cameraserver:cameraserver-java:$wpiVersion")
    api("edu.wpi.first.wpilibNewCommands:wpilibNewCommands-java:$wpiVersion")

    val ctreVersion = "5.20.2"
    api("com.ctre.phoenix:wpiapi-java:$ctreVersion")
    api("com.ctre.phoenix:api-java:$ctreVersion")

    val revVersion = "2022.1.1"
    api("com.revrobotics.frc:REVLib-java:$revVersion")
}

configure<LicenseExtension> {
    header = rootProject.file("HEADER.txt")
    (this as ExtensionAware).extra.apply {
        set("name", rootProject.name)
        for (key in listOf("organization", "url")) {
            set(key, rootProject.property(key))
        }
    }
}

release {
    tagTemplate = "v\${version}"
    buildTasks = listOf("build")
}

publishing {
    publications {
        register<MavenPublication>("library") {
            from(components["java"])
        }
    }
    repositories {
        maven {
            val releasesRepoUrl = "https://maven.octyl.net/repository/team5818-releases"
            val snapshotsRepoUrl = "https://maven.octyl.net/repository/team5818-snapshots"
            name = "octylNet"
            url = uri(if (version.toString().endsWith("SNAPSHOT")) snapshotsRepoUrl else releasesRepoUrl)
            credentials(PasswordCredentials::class)
        }
    }
}