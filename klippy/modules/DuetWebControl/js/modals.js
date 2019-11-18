/* Modal dialog functions for Duet Web Control
 * 
 * written by Christian Hammacher (c) 2016-2017
 * 
 * licensed under the terms of the GPL v3
 * see http://www.gnu.org/licenses/gpl-3.0.html
 */


/* Generic workarounds */

$(".modal").on("hidden.bs.modal", function() {
	// Bootstrap bug: Padding is added to the right, but never cleaned
	$("body").css("padding-right", "");
});


/* Confirmation Dialog */

function showConfirmationDialog(title, message, callback) {
	$("#modal_confirmation h4").html('<span class="glyphicon glyphicon-question-sign"></span> ' + title);
	$("#modal_confirmation p").html(message);
	$("#modal_confirmation button.btn-success").off().one("click", callback);
	$("#modal_confirmation").modal("show");
	$("#modal_confirmation .btn-success").focus();
}


/* Text Input Dialog */

function showTextInput(title, message, callback, text, emptyCallback) {
	$("#modal_textinput h4").html(title);
	$("#modal_textinput p").html(message);
	$("#modal_textinput input").val((text == undefined) ? "" : text);
	$("#modal_textinput form").off().submit(function(e) {
		$("#modal_textinput").modal("hide");
		var value = $("#modal_textinput input").val();
		if (value.trim() != "") {
			callback(value);
		} else if (emptyCallback != undefined) {
			emptyCallback();
		}
		e.preventDefault();
	});
	$("#modal_textinput").modal("show");
}

$("#modal_textinput").on("shown.bs.modal", function() {
	$("#modal_textinput input").focus();
});


/* Change Step Dialog */

var stepChangeType, stepChangeIndex, stepChangeAxisIndex;

function showStepDialog(type, index, currentValue, axisIndex) {
	stepChangeType = type;
	stepChangeIndex = index;
	stepChangeAxisIndex = axisIndex;

	$("#input_step_amount").val(currentValue);
	$("#input_step_amount_unit").text((type == "feedrate") ? T("mm/s") : T("mm"));
	$("#modal_change_step").modal("show");
}

$("#modal_change_step").on("shown.bs.modal", function() {
	$("#modal_change_step input").focus();
});

$("#modal_change_step form").submit(function(e) {
	if (stepChangeType == "axis") {
		settings.axisMoveSteps[stepChangeAxisIndex][stepChangeIndex] = $("#input_step_amount").val();
	} else if (stepChangeType == "amount") {
		settings.extruderAmounts[stepChangeIndex] = $("#input_step_amount").val();
	} else if (stepChangeType == "offset") {
		settings.offsetAmounts[stepChangeIndex] = $("#input_step_amount").val();
	} else {
		settings.extruderFeedrates[stepChangeIndex] = $("#input_step_amount").val();
	}

	applyMovementSteps();
	$("#modal_change_step").modal("hide");

	e.preventDefault();
});


/* Host Prompt */

function showHostPrompt() {
	$('#input_host').val(getLocalSetting("lastHost", ""));
	$("#modal_host_input").modal("show");
}

$("#modal_host_input").on("shown.bs.modal", function() {
	$("#input_host").focus();
});

$("#form_host").submit(function(e) {
	$("#modal_host_input").off("hide.bs.modal").modal("hide");
	ajaxPrefix = settings.lastHost = $("#input_host").val();		// let the user decide if this shall be saved

	ajaxPrefix += "/";
	if (ajaxPrefix.indexOf("://") == -1) {
		// Prepend http prefix if no URI scheme is given
		ajaxPrefix = "http://" + ajaxPrefix;
	}

	connect(sessionPassword, false);
	e.preventDefault();
});


/* Password prompt */

function showPasswordPrompt() {
	$('#input_password').val("");
	$("#modal_pass_input").modal("show");
}

$("#form_password").submit(function(e) {
	$("#modal_pass_input").off("hide.bs.modal").modal("hide");
	connect($("#input_password").val(), false);
	e.preventDefault();
});

$("#modal_pass_input").on("shown.bs.modal", function() {
	$("#input_password").focus();
});


/* Filament Change Dialog */

var filamentChangeTool, changingFilament;

function showFilamentDialog(tool, changeFilament) {
	// make list of all available filaments
	$("#div_filaments").children().remove();
	$("#table_filaments > tbody > tr").each(function() {
		var filament = $(this).data("filament");
		var isLoaded = false;
		for(var i = 0; i < toolMapping.length; i++) {
			if (toolMapping[i].hasOwnProperty("filament") && toolMapping[i].filament == filament) {
				isLoaded = true;
				break;
			}
		}

		if (!isLoaded) {
			$("#div_filaments").append('<a href="#" class="a-load-filament list-group-item" data-filament="' + filament + '"><span class="glyphicon glyphicon-cd"></span> ' + filament + '</a>');
		}
	});

	// show notification or selection dialog
	if ($("#div_filaments").children().length == 0) {
		showMessage("warning", T("No Filaments"), T("There are no other filaments available to choose. Please go to the Filaments page and define more."));
	} else {
		filamentChangeTool = tool;
		changingFilament = changeFilament;
		$("#modal_change_filament").modal("show");
	}
}

$("body").on("click", ".a-load-filament", function(e) {
	$("#modal_change_filament").modal("hide");

	var gcode = "";
	if (lastStatusResponse != undefined && lastStatusResponse.currentTool != filamentChangeTool) {
		gcode = "T" + filamentChangeTool + "\n";
	}
	if (changingFilament) {
		gcode += "M702\n";
	}
	gcode += "M701 S\"" + $(this).data("filament") + "\"";
	if (!compatibilityMode) { gcode += "\nM703"; }
	sendGCode(gcode);

	e.preventDefault();
});


/* File Edit Dialog */

function showEditDialog(title, content, callback) {
	$("#modal_edit .modal-title").text(T("Editing {0}", title));
	var edit = $("#modal_edit textarea").val(content).get(0);
	edit.selectionStart = edit.selectionEnd = 0;
	$("#modal_edit").modal("show");
	$("#btn_save_file").off("click").click(function() {
		$("#modal_edit").modal("hide");
		callback($("#modal_edit textarea").val());
	});
}

$("#modal_edit").on("shown.bs.modal", function() {
	$("#modal_edit textarea").focus();
});

$("#modal_edit div.modal-content").resize(function() {
	var contentHeight = $(this).height();
	var headerHeight = $("#modal_edit div.modal-header").height();
	var footerHeight = $("#modal_edit div.modal-footer").height();
	$("#modal_edit div.modal-body").css("height", (contentHeight - headerHeight - footerHeight - 60) + "px");
});

$(document).delegate("#modal_edit textarea", "keydown", function(e) {
	var keyCode = e.keyCode || e.which;

	if (keyCode == 9) {
		e.preventDefault();
		var start = $(this).get(0).selectionStart;
		var end = $(this).get(0).selectionEnd;

		// set textarea value to: text before caret + tab + text after caret
		$(this).val($(this).val().substring(0, start)
				+ "\t"
				+ $(this).val().substring(end));

		// put caret at right position again
		$(this).get(0).selectionStart = $(this).get(0).selectionEnd = start + 1;
	}
});


/* Start Scan Dialog (proprietary) */

$("#btn_start_scan").click(function() {
	if (!$(this).hasClass("disabled")) {
		if (vendor == "diabase") {
			// Properietary implemenation with extra steps
			$("#modal_start_scan").modal("show");
		} else {
			// Basic open-source variant
			showTextInput(T("Start new 3D scan"), T("Please enter a name for the new scan:"), function(name) {
				if (filenameValid(name)) {
					// Let the firmware do the communication to the board
					sendGCode("M752 S360 P" + name);
				} else {
					showMessage("danger", T("Error"), T("The specified filename is invalid. It may not contain quotes, colons or (back)slashes."));
				}
			}, undefined, function() {
				showMessage("danger", T("Error"), T("The filename for a new scan must not be empty!"));
			});
		}
	}
});

$("input[name='scanMode']").change(function() {
	$(".scan-unit").text(($("input[name='scanMode']:checked").val() == 0) ? T("mm") : T("°"));
});

$("#modal_start_scan input").keyup(function() {
	$("#btn_start_scan_modal").toggleClass("disabled", $("#modal_start_scan input:invalid").length > 0);
});

$("#btn_toggle_laser").click(function(e) {
	if ($(this).hasClass("active")) {
		sendGCode("M755 P0");
		$(this).removeClass("active").children("span.content").text(T("Activate Laser"));
	} else {
		sendGCode("M755 P1")
		$(this).addClass("active").children("span.content").text(T("Deactivate Laser"));
	}

	$(this).blur();
	e.preventDefault();
});

$("#modal_start_scan form").submit(function(e) {
	if (!$("#btn_start_scan_modal").hasClass("disabled")) {
		// 1. Turn off the alignment laser if it is still active
		if ($("#btn_toggle_laser").hasClass("active")) {
			$("#btn_toggle_laser").removeClass("active").children("span.content").text(T("Activate Laser"));
			if (isConnected) {
				sendGCode("M755 P0");
			}
		}

		// 2. Start a new scan
		var filename = $("#input_scan_filename").val();
		var range = $("#input_scan_range").val();
		var resolution = $("#input_scan_resolution").val();
		var mode = $("input[name='scanMode']:checked").val();

		// 3. Check the filename and start a new scan
		if (filenameValid(filename)) {
			sendGCode("M752 S" + range + " R" + resolution + " N" + mode + " P\"" + filename + "\"");
		} else {
			showMessage("danger", T("Error"), T("The specified filename is invalid. It may not contain quotes, colons or (back)slashes."));
		}

		// 4. Hide the modal dialog
		$("#modal_start_scan").modal("hide");
	}

	e.preventDefault();
});

$("#modal_start_scan").on("hidden.bs.modal", function() {
	if ($("#btn_toggle_laser").hasClass("active")) {
		$("#btn_toggle_laser").removeClass("active").children("span.content").text(T("Activate Laser"));
		if (isConnected) {
			// Turn off laser again when the dialog is closed
			sendGCode("M755 P0");
		}
	}
});


/* Scanner Progress Dialogs */

function updateScannerDialogs(scanResponse) {
	var scanProgress = 100, postProcessingProgress = 100, uploadProgress = 100;

	if (scanResponse.status == "S" || scanResponse.status == "P" || scanResponse.status == "U") {
		// Scanner is active
		if (!$("#modal_scanner").hasClass("in")) {
			$("#btn_cancel_scan").removeClass("hidden").removeClass("disabled");
			$("#btn_close_scan").addClass("hidden");

			$("#modal_scanner .modal-title").text(T("Scanning..."));
			$("#p_scan_info").text(T("Please wait while a scan is being made. This may take a while..."));

			$("#modal_scanner").modal("show");
		}

		// Update progress
		if (scanResponse.status == "S") {
			scanProgress = scanResponse.progress;
			postProcessingProgress = 0;
			uploadProgress = 0;
		} else if (scanResponse.status == "P") {
			scanProgress = 100;
			postProcessingProgress = scanResponse.progress;
			uploadProgress = 0;
		} else if (scanResponse.status == "U") {
			scanProgress = 100;
			postProcessingProgress = 100;
			uploadProgress = scanResponse.progress;
			$("#btn_cancel_scan").addClass("disabled");
		}
	} else if (scanResponse.status == "C") {
		// Scanner calibration is running
		if (!$("#modal_scanner_calibration").hasClass("in")) {
			$("#btn_cancel_calibration").removeClass("disabled");
			$("#modal_scanner_calibration").modal("show");
		}

		// Update progress
		$("#progress_calibration").css("width", scanResponse.progress + "%");
		$("#span_calibration_progress").text(T("{0} %", scanResponse.progress));
	} else if (scanResponse.status == "I") {
		// Scanner is inactive
		if ($("#modal_scanner").hasClass("in") && $("#modal_scanner .modal-title").text() != T("Scan complete")) {
			$("#modal_scanner .modal-title").text(T("Scan complete"));
			$("#p_scan_info").text(T("Your 3D scan is now complete! You may download it from the file list next."));

			$("#btn_cancel_scan").addClass("hidden");
			$("#btn_close_scan").removeClass("hidden");

			updateScanFiles();
			$(".span-refresh-scans").addClass("hidden");
		}

		if ($("#modal_scanner_calibration").hasClass("in")) {
			$("#modal_scanner_calibration").modal("hide");
		}
	}

	// Update progress bars
	if ($("#modal_scanner").hasClass("in")) {
		$("#progress_scan").css("width", scanProgress + "%");
		$("#span_scan_progress").text(T("{0} %", scanProgress));

		$("#progress_scan_postprocessing").css("width", postProcessingProgress + "%");
		$("#span_scan_postprocessing_progress").text(T("{0} %", postProcessingProgress));

		$("#progress_scan_upload").css("width", uploadProgress + "%");
		$("#span_scan_upload_progress").text(T("{0} %", uploadProgress));
	}
}

$("#btn_cancel_scan").click(function() {
	if (!$(this).hasClass("disabled")) {
		sendGCode("M753");
		$("#modal_scanner").modal("hide");
	}
});

$("#btn_cancel_calibration").click(function() {
	if (!$(this).hasClass("disabled")) {
		sendGCode("M753");
		$(this).addClass("disabled");
	}
});


/* Message Box Dialog */

var messageBoxResponse = undefined;

function updateMessageBox(response) {
	var timeout = response.timeout;
	response.timeout = 0;

	var stringifiedResponse = JSON.stringify(response);
	if (stringifiedResponse != messageBoxResponse)
	{
		messageBoxResponse = stringifiedResponse;
		showMessageBox(response.msg, response.title, response.mode, timeout, response.controls);
	}
}

function closeMessageBox() {
	if (messageBoxResponse != undefined) {
		if ($("#modal_messagebox").hasClass("in")) {
			$("#modal_messagebox").modal("hide");
		}
		messageBoxResponse = undefined;
	}
}


var messageBoxTimer = undefined;

function showMessageBox(message, title, mode, timeout, controls) {
	// Display message, title and optionally show Z controls
	$("#h3_messagebox").html(message);
	$("#h4_messagebox_title").html(title);
	$("#modal_messagebox div.modal-header").toggleClass("hidden", title == "");

	// Toggle axis control visibility
	$("#div_x_controls").toggleClass("hidden", (controls & (1 << 0)) == 0);
	$("#div_y_controls").toggleClass("hidden", (controls & (1 << 1)) == 0);
	$("#div_z_controls").toggleClass("hidden", (controls & (1 << 2)) == 0);

	// Toggle button visibility
	$("#modal_messagebox div.modal-footer").toggleClass("hidden", mode == 0);
	$("#modal_messagebox [data-dismiss]").toggleClass("hidden", mode != 1);
	$("#btn_ack_messagebox").toggleClass("hidden", mode == 0 || mode == 1);
	$("#btn_cancel_messagebox").toggleClass("hidden", mode != 3);

	// Show message box
	var backdropValue = (mode != 1) ? "static" : "true";
	$("#modal_messagebox").modal({ backdrop: backdropValue });

	var data = $("#modal_messagebox").data("bs.modal");
	data.options.backdrop = backdropValue;
	$("#modal_messagebox").data("bs.modal", data);

	// Take care of the timeouts
	if (messageBoxTimer != undefined) {
		clearTimeout(messageBoxTimer);
	}
	if (timeout > 0) {
		messageBoxTimer = setTimeout(function() {
			messageBoxTimer = undefined;
			$("#modal_messagebox").modal("hide");
		}, timeout * 1000);
	}
}

$("#btn_ack_messagebox").click(function() {
	$("#modal_messagebox").modal("hide");
	sendGCode("M292");
});

$("#btn_cancel_messagebox").click(function() {
	$("#modal_messagebox").modal("hide");
	sendGCode("M292 P1");
});

$('#modal_messagebox').on("hide.bs.modal", function() {
	if (messageBoxTimer != undefined) {
		clearTimeout(messageBoxTimer);
		messageBoxTimer = undefined;
	}

	// FIXME: This is needed to ensure the backdrop always works as intended
	$('#modal_messagebox').removeData();
});

/* Set mesh grid area */

$("#a_define_mesh").click(function(e) {
	$("#div_mesh_x, #div_mesh_y").toggleClass("hidden", geometry == "delta");
	$("#div_mesh_delta").toggleClass("hidden", geometry != "delta");
	$("#modal_define_mesh").modal("show");
	e.preventDefault();
});


$("#modal_define_mesh form").submit(function(e) {
	if (geometry != "delta") {
		var minX = $("#input_mesh_x_min").val();
		var maxX = $("#input_mesh_x_max").val();
		var spacingX = $("#input_mesh_x_spacing").val();
		var minY = $("#input_mesh_y_min").val();
		var maxY = $("#input_mesh_y_max").val();
		var spacingY = $("#input_mesh_y_spacing").val();
		sendGCode("M557 X" + minX + ":" + maxX + " Y" + minY + ":" + maxY + " S" + spacingX + ":" + spacingY);
	} else {
		var radius = $("#input_mesh_radius").val();
		var spacing = $("#input_mesh_spacing").val();
		sendGCode("M557 R" + radius + " S" + spacing);
	}

	$("#modal_define_mesh").modal("hide");
	e.preventDefault();
});

/* Probe Cylinder (OEM) */

$("#a_probe_cylinder").click(function(e) {
	$("#modal_cylinder").modal("show");
	e.preventDefault();
});

$("#modal_cylinder form").submit(function(e) {
	sendGCode("G28 Z\nG92 Z" + ($("#input_cylinder_diameter").val() / 2));
	$("#modal_cylinder").modal("hide");
	e.preventDefault();
});
