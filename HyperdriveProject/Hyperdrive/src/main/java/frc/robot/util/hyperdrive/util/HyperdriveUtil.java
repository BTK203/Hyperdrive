package frc.robot.util.hyperdrive.util;

import java.io.IOException;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Iterator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;

/** 
 * This class is similar to Utilities in Monopoly, because similarly to the fact that the Utilities are the worst property in Monopoly,
 * this class is the worst in this library.
 */
public class HyperdriveUtil {
	//This constructor exists so that no one can create an instance of this class.
	private HyperdriveUtil() {
	}

	/**
	 *	Kind of self explanatory, but with some spice
	 *	Use this mainly as a get method to retrieve values the user types into the smart dash
	 *	(the 'Set' part is only in case the value doesn't exist, backup is a default to use and set if it isn't there)
	 * @param key The name of the value
	 * @param backup The value to set if the value does not exist
	 */
	static Preferences pref = Preferences.getInstance();
	public static double getAndSetDouble(String key, double backup) {
		if(!pref.containsKey(key)) pref.putDouble(key, backup);
		return pref.getDouble(key, backup);
	}

	/**
     * Really stupid but needed to round a double to n places
     * @param value  original value
     * @param places how many values after decimal point
     * @return       rounded value
     */
    public static double roundTo(double value, int places) {
        double val = value;
        val *= Math.pow(10, places);
        val = Math.round(val);
        val /= Math.pow(10, places);
        return val;
	}

	/**
	 * Converts a measurement in some unit to inches.
	 * @param value The value of the measurement to convert
	 * @param units The original units of the measurement.
	 * @return The value of the measurement in inches.
	 */
	private static double toInches(double value, Units.LENGTH units) {
		switch(units) {
		case CENTIMETERS:
			return value / 2.54;
		case FEET:
			return value * 12;
		case INCHES:
			return value;
		case METERS:
			return value / 0.0254;
		case YARDS:
			return value / 0.027778;
		default:
			return value;
		}
	}

	/**
	 * Converts a measurement in inches to some unit.
	 * @param value The value of the measurement in inches.
	 * @param desired The units to convert the measurement to.
	 * @return The value of the measurement in the desired unit.
	 */
	private static double fromInches(double value, Units.LENGTH desired) {
		switch(desired) {
		case CENTIMETERS:
			return value * 2.54;
		case FEET:
			return value / 12;
		case INCHES:
			return value;
		case METERS:
			return value * 0.0254;
		case YARDS:
			return value * 0.027778;
		default:
			return value;
		}
	}

	/**
	 * Converts a measurement between units of length.
	 * @param value The value of the measurement to convert.
	 * @param original The current unit of the measurement.
	 * @param desired The unit to convert the measurement to.
	 * @return The value of the measurement in the desired unit.
	 */
	public static double convertDistance(double value, Units.LENGTH original, Units.LENGTH desired) {
		double inches = toInches(value, original);
		return fromInches(inches, desired);
	}

	/**
	 * Converts a time value in some unit to seconds.
	 * @param value The original value.
	 * @param original The time units of the original value.
	 * @return The value of the time in seconds.
	 */
	private static double toSeconds(double value, Units.TIME original) {
		switch(original) {
		case DECASECONDS:
			return value / 10;
		case MINUTES:
			return value / 0.01666666667;
		case SECONDS:
			return value;
		default:
			return value;
		}
	}

	/**
	 * Converts a time value in seconds to some unit.
	 * @param value The original value in seconds
	 * @param desired The units to convert to.
	 * @return The time value, converted to the new units.
	 */
	private static double fromSeconds(double value, Units.TIME desired) {
		switch(desired) {
		case DECASECONDS:
			return value * 10;
		case MINUTES:
			return value * 0.01666666667;
		case SECONDS:
			return value;
		default:
			return value;
		}
	}

	/**
	 * Converts a value between units of time.
	 * @param value The value to convert between.
	 * @param original The original units of the value.
	 * @param desired The units to convert the value to.
	 * @return The value, converted to the new units.
	 */
	public static double convertTime(double value, Units.TIME original, Units.TIME desired) {
		double seconds = toSeconds(value, original);
		return fromSeconds(seconds, desired);
	}

	/**
	 * Converts a force value in some unit to Newtons.
	 * @param value The original force value.
	 * @param original The units of the original value.
	 * @return The value of the force in the new units.
	 */
	private static double toNewtons(double value, Units.FORCE original) {
		switch(original) {
		case NEWTON:
			return value;
		case POUND:
			return value * 4.44822;
		case KILOGRAM_FORCE:
			return value * 9.80665;
		default:
			return value;
		}
	}

	/**
	 * Converts a force value in Newtons to some unit.
	 * @param value The original force value in Newtons.
	 * @param desired The unit to convert the force to.
	 * @return The value of the force in the new units.
	 */
	private static double fromNewtons(double value, Units.FORCE desired) {
		switch(desired) {
		case NEWTON:
			return value;
		case POUND:
			return value / 4.44822;
		case KILOGRAM_FORCE:
			return value / 9.80665;
		default:
			return value;
		}
	}

	/**
	 * Converts a force value between units.
	 * @param value The value of the original force.
	 * @param original The units of the force.
	 * @param desired The unit to convert to.
	 * @return The value of the original force in the new units.
	 */
	public static double convertForce(double value, Units.FORCE original, Units.FORCE desired) {
		double newtons = toNewtons(value, original);
		return fromNewtons(newtons, desired);
	}

	/**
	 * Calculates the mass of an object, in kilograms, from its weight.
	 * @param weight The weight of the object, in some unit.
	 * @param units The units of the weight.
	 * @return The mass of the object, in KG.
	 */
	public static double massKGFromWeight(double weight, Units.FORCE units) {
		double newtons = convertForce(weight, units, Units.FORCE.NEWTON);
		return newtons / 9.80665;
	}

	/**
	 * Calculates the weight of an object based on its mass.
	 * @param mass The mass of the object.
	 * @param desiredWeightUnit The unit of weight to return.
	 * @return The weight of the object, in the desired unit.
	 */
	public static double weightFromMassKG(double mass, Units.FORCE desiredWeightUnit) {
		double newtons = mass * 9.80665;
		return convertForce(newtons, Units.FORCE.NEWTON, desiredWeightUnit);
	}

    /**
     * Returns the number in the set that is closest to zero.
     * @param set An array of doubles to parse
     * @return The value of the number in the set with the least absolute value.
     */
	public static double closestToZero(double[] set) {
		double least = Double.MAX_VALUE;
		for(int i=0; i<set.length; i++) {
			if(Math.abs(set[i]) < Math.abs(least)) {
				least = set[i];
			}
		}

		return least;
	}

	/**
	 * Gets the angle that the robot needs to turn through to achieve a heading.
	 * @param heading1   The angle of the robot
	 * @param heading2 The desired heading to be achieved.
	 * @return        The angle that the robot needs to turn to have its desired heading.
	 */
	public static double getAngleBetweenHeadings(double heading1, double heading2) {
		double angle1 = (heading2 - heading1) % 360;         //angle to heading without crossing 0
		double angle2 = angle1 - 360;
		double angle3 = angle1 + 360;

		return closestToZero(new double[] {angle1, angle2, angle3});
	}

	/**
	 * Calculates the exact deviance between two points. This method takes headings into account, meaning
	 * that a point that is two units directly in front of the target point will return 0, because
	 * the current point is still directly in line with the target.
	 * @param current The point to calculate the deviance of
	 * @param target The point to calculate the deviance relative to.
	 * @return The deviance of the current point to the target point.
	 */
	public static double getDeviance(Point2D current, Point2D target) {
		//define a triangle connecting the current point, target point, and line perpendicular to the target's heading.
		double
			angleToCurrent = Math.abs(target.getHeadingTo(current)),
			angleToTarget = Math.abs(current.getHeadingTo(target)),
			// angleToCurrent = target.getHeadingTo(current),
			// angleToTarget = current.getHeadingTo(target),
			a = 90 - (angleToCurrent - target.getHeading()),
			b = 180 - (angleToTarget - current.getHeading()),
			c = 180 - b - a,
			actualDistance = current.getDistanceFrom(target);

		System.out.println("current: " + current);
		System.out.println("target: " + target);
		System.out.println("angle to current: " + angleToCurrent);
		System.out.println("angle to target: " + angleToTarget);
		System.out.println("a: " + a);
		System.out.println("b: " + b);
		System.out.println("c: " + c);
		System.out.println("actual distance: " + actualDistance);

		//convert b and c to radians for the java math library
		b = Math.toRadians(b);
		c = Math.toRadians(c);

		//use law of sines to get the deviance
		double deviance = (actualDistance * Math.sin(b)) / Math.sin(c);
		System.out.println("deviance: " + deviance);
		System.out.println("sinb: " + Math.sin(b));
		System.out.println();
		return Math.abs(deviance);
	}

	/**
	 * Returns a list of files.
	 * @param directory The directory to search
	 * @param specifyDirectories If true, add a tag specifying whether or not a file is a directory.
	 * @return A list of files and directories.
	 */
	public static String[] getFilesInDirectory(String directory, boolean specifyDirectories) {
        try {
            ArrayList<String> paths = new ArrayList<String>();
            DirectoryStream<java.nio.file.Path> stream = Files.newDirectoryStream(java.nio.file.Path.of(directory));
            Iterator<java.nio.file.Path> iterator = stream.iterator();
            while(iterator.hasNext()) {
                paths.add(iterator.next().toAbsolutePath().toString());
            }
            stream.close();

            //put stuff in string array
            String[] contents = new String[paths.size()];
            for(int i=0; i<paths.size(); i++) {
                String path = paths.get(i);
                if(specifyDirectories) {
                    if(Files.isDirectory(java.nio.file.Path.of(path))) {
                        path += ":dir";
                    } else {
                        path += ":file";
                    }
                }
                contents[i] = path;
            }

            return contents;
        } catch(IOException ex) {
            DriverStation.reportWarning("Tried to look in directory \"" + directory + "\" but encountered problems.", true);
            return new String[0];
        }
    }

	/**
	 * Loads values from a file into a double array.
	 * @param filename The name of the file to load from.
	 * @return An array containing the values in the array.
	 */
	public static double[] loadValuesFromFile(String filename) {
		try {
			String contents = Files.readString(java.nio.file.Path.of(filename));
			contents = contents.replace('\n', ' ');
			String[] split = contents.split(" ");
			
			//load values into program as doubles
			ArrayList<Double> valuesList = new ArrayList<Double>();
			for(int i=0; i<split.length; i++) {
				try {
					valuesList.add(Double.valueOf(split[i]));
				} catch(NumberFormatException ex) {
				}
			}

			//convert valuesList to an array
			double[] valuesToReturn = new double[valuesList.size()];
			for(int i=0; i<valuesList.size(); i++) {
				valuesToReturn[i] = valuesList.get(i);
			}

			return valuesToReturn;
		} catch(IOException ex) {
			DriverStation.reportError("Could not read values from file " + filename + "!", true);
			return new double[0];
		}

	}

	/**
	 * Saves double values to a file.
	 * @param values The values to save.
	 * @param fileName The name of the file to save the values to.
	 */
	public static void saveValuesToFile(double[] values, String fileName) {
		String contentsToWrite = "";
		for(int i=0; i<values.length; i++) {
			contentsToWrite += Double.valueOf(values[i]).toString() + "\n";
		}

		try {
			Files.writeString(java.nio.file.Path.of(fileName), contentsToWrite);
		} catch(IOException ex) {
			DriverStation.reportError("Could not write to file " + fileName + "!\nError: " + ex.getMessage(), true);
		}
	}
}